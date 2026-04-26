#!/usr/bin/env python3
"""Viser ghost-rollout viewer for the G1 visual-backbone residual controller.

Three robots in one scene:
  /real_robot          live state from /lowstate + /sportmodestate (white)
  /ghost_robot         pure motion reference / "demo" from controller buffer (green)
  /residual_robot      real_q + residual delta, anchored at real root (blue)

Plus an /object_goal world-frame axes marker and a body-frame
/hlc_goal_in_body marker showing what the HLC's observation actually encodes.

Usage:
  ros2 run cpp_control viser_ghost_g1_residual.py \\
      --urdf /home/lkrajan/Downloads/unitree_description/urdf/g1/main.urdf
"""

from __future__ import annotations

import argparse
import threading
import time

import numpy as np
import rclpy
import yourdfpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped

import viser
from viser.extras import ViserUrdf

from unitree_hg.msg import LowState as HGLowState
from unitree_go.msg import SportModeState


# ── Joint orderings — copied verbatim from g1_residual.cpp ────────────────────
IL_JOINTS = [
    "left_hip_pitch_joint",     "right_hip_pitch_joint",     "waist_yaw_joint",
    "left_hip_roll_joint",      "right_hip_roll_joint",      "waist_roll_joint",
    "left_hip_yaw_joint",       "right_hip_yaw_joint",       "waist_pitch_joint",
    "left_knee_joint",          "right_knee_joint",
    "left_shoulder_pitch_joint","right_shoulder_pitch_joint",
    "left_ankle_pitch_joint",   "right_ankle_pitch_joint",
    "left_shoulder_roll_joint", "right_shoulder_roll_joint",
    "left_ankle_roll_joint",    "right_ankle_roll_joint",
    "left_shoulder_yaw_joint",  "right_shoulder_yaw_joint",
    "left_elbow_joint",         "right_elbow_joint",
    "left_wrist_roll_joint",    "right_wrist_roll_joint",
    "left_wrist_pitch_joint",   "right_wrist_pitch_joint",
    "left_wrist_yaw_joint",     "right_wrist_yaw_joint",
]
MJ_JOINTS = [
    "left_hip_pitch_joint",     "left_hip_roll_joint",       "left_hip_yaw_joint",
    "left_knee_joint",          "left_ankle_pitch_joint",    "left_ankle_roll_joint",
    "right_hip_pitch_joint",    "right_hip_roll_joint",      "right_hip_yaw_joint",
    "right_knee_joint",         "right_ankle_pitch_joint",   "right_ankle_roll_joint",
    "waist_yaw_joint",          "waist_roll_joint",          "waist_pitch_joint",
    "left_shoulder_pitch_joint","left_shoulder_roll_joint",   "left_shoulder_yaw_joint",
    "left_elbow_joint",         "left_wrist_roll_joint",     "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint","right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint",        "right_wrist_roll_joint",    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]
NQ = 29
assert len(IL_JOINTS) == NQ and len(MJ_JOINTS) == NQ

DEMO_RGBA     = (0.4, 0.95, 0.5, 0.55)   # green: pure motion reference (the "demo")
RESIDUAL_RGBA = (0.4, 0.7, 1.0, 0.55)    # blue:  real + residual delta
PARK_Z = -100.0          # send stale ghosts to the basement
STALE_AFTER = 0.4        # seconds without an update → considered stale


def rot6d_to_quat_wxyz(r6d: np.ndarray) -> np.ndarray:
    """Inverse of obs::append_rotation_6d (Zhou et al. 6D representation)."""
    a1 = r6d[0:3].astype(np.float64)
    a2 = r6d[3:6].astype(np.float64)
    b1 = a1 / max(np.linalg.norm(a1), 1e-9)
    a2_proj = a2 - np.dot(b1, a2) * b1
    b2 = a2_proj / max(np.linalg.norm(a2_proj), 1e-9)
    b3 = np.cross(b1, b2)
    R = np.stack([b1, b2, b3], axis=1)
    t = R.trace()
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = 2.0 * np.sqrt(max(1.0 + R[0, 0] - R[1, 1] - R[2, 2], 1e-9))
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = 2.0 * np.sqrt(max(1.0 + R[1, 1] - R[0, 0] - R[2, 2], 1e-9))
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(max(1.0 + R[2, 2] - R[0, 0] - R[1, 1], 1e-9))
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    return q / max(np.linalg.norm(q), 1e-9)


class ViserGhostG1(Node):
    def __init__(self, urdf_path: str, port: int):
        super().__init__("viser_ghost_g1_residual")

        self._urdf = yourdfpy.URDF.load(urdf_path)
        urdf_joints = [j.name for j in self._urdf.robot.joints if j.type != "fixed"]

        self._urdf_to_mj = np.array(
            [MJ_JOINTS.index(n) if n in MJ_JOINTS else -1 for n in urdf_joints], dtype=np.int64
        )
        self._urdf_to_il = np.array(
            [IL_JOINTS.index(n) if n in IL_JOINTS else -1 for n in urdf_joints], dtype=np.int64
        )
        self._n_urdf = len(urdf_joints)

        # ── viser ────────────────────────────────────────────────
        self._server = viser.ViserServer(port=port, label="g1 residual ghost stream")
        self._server.gui.configure_theme(dark_mode=True, control_layout="collapsible")
        self._server.scene.add_grid(
            "/ground",
            infinite_grid=True,
            cell_color=(40, 40, 40),
            section_color=(80, 80, 80),
            cell_size=0.5,
            section_size=2.0,
            fade_distance=200.0,
            plane_color=(10, 10, 10),
            plane_opacity=1.0,
        )

        # reality (white)
        self._real_root = self._server.scene.add_frame("/real_robot", show_axes=False)
        self._real_urdf = ViserUrdf(self._server, self._urdf, root_node_name="/real_robot")
        # demo: pure motion reference (green)
        self._ghost_root = self._server.scene.add_frame(
            "/ghost_robot", show_axes=False, position=(0.0, 0.0, PARK_Z)
        )
        self._ghost_urdf = ViserUrdf(
            self._server, self._urdf, root_node_name="/ghost_robot",
            mesh_color_override=DEMO_RGBA,
        )
        # residual overlay: real_q + residual delta, anchored at real root (blue)
        self._residual_root = self._server.scene.add_frame(
            "/residual_robot", show_axes=False, position=(0.0, 0.0, PARK_Z)
        )
        self._residual_urdf = ViserUrdf(
            self._server, self._urdf, root_node_name="/residual_robot",
            mesh_color_override=RESIDUAL_RGBA,
        )
        # object goal pose in world frame
        self._goal_root = self._server.scene.add_frame(
            "/object_goal", show_axes=True, axes_length=0.2, axes_radius=0.01
        )
        self._server.scene.add_label("/object_goal/label", 
                                     text="object_goal_w",
                                     position=(0.0, 0.0, 0.15))
        # Anchor frame: sits at world origin, rotated by the robot's IMU quat.
        # This matches the C++ HLC obs definition (robot_pos hardcoded to {0,0,0}),
        # so anchor_to_world = (origin, imu_quat). The goal child below carries
        # body-relative coords; rendered through this parent it should overlay
        # with /object_goal iff the obs slot is consistent.
        self._anchor_root = self._server.scene.add_frame(
            "/anchor_frame", 
            show_axes=True, axes_length=0.2, axes_radius=0.01
        )
        self._server.scene.add_label("/anchor_frame/label",
                                     text="anchor_frame_w",
                                     position=(0.0, 0.0, 0.15))
        self._hlc_goal_root = self._server.scene.add_frame(
            "/anchor_frame/object_goal_pose_anchor",
            show_axes=True, axes_length=0.15, axes_radius=0.008,
        )

        # ── GUI: visibility toggles ──────────────────────────────
        with self._server.gui.add_folder("layers"):
            self._show_real     = self._server.gui.add_checkbox("reality",                       True)
            self._show_ghost    = self._server.gui.add_checkbox("demo (green)",                  True)
            self._show_residual = self._server.gui.add_checkbox("residual overlay (blue)",      True)
            self._show_goal     = self._server.gui.add_checkbox("object_goal_pose_world",       True)
            self._show_hlc      = self._server.gui.add_checkbox("object_goal_pose_anchor",      True)

            @self._show_real.on_update
            def _(_): self._real_root.visible = self._show_real.value
            @self._show_ghost.on_update
            def _(_): self._ghost_root.visible = self._show_ghost.value
            @self._show_residual.on_update
            def _(_): self._residual_root.visible = self._show_residual.value
            @self._show_goal.on_update
            def _(_): self._goal_root.visible = self._show_goal.value
            @self._show_hlc.on_update
            def _(_): self._hlc_goal_root.visible = self._show_hlc.value

        # ── shared state ─────────────────────────────────────────
        self._lock = threading.Lock()
        self._real_joint_pos_mj: np.ndarray | None = None
        self._real_imu_quat: np.ndarray | None = None
        self._real_world_pos: np.ndarray = np.zeros(3, dtype=np.float64)
        self._ghost_joint_pos_il: np.ndarray | None = None
        self._ghost_anchor_pos: np.ndarray | None = None
        self._ghost_anchor_quat: np.ndarray | None = None
        self._residual_term_mj: np.ndarray | None = None  # latest residual delta (MJ)
        self._goal_pos: np.ndarray | None = None
        self._goal_quat: np.ndarray | None = None
        self._hlc_goal_pos_b: np.ndarray | None = None
        self._hlc_goal_quat_b: np.ndarray | None = None
        # last-receipt monotonic timestamps for staleness checks
        self._ghost_last_t    = 0.0
        self._residual_last_t = 0.0
        self._hlc_last_t      = 0.0

        # ── ROS subscriptions ────────────────────────────────────
        self.create_subscription(HGLowState, "/lowstate", self._on_lowstate, 10)
        self.create_subscription(SportModeState, "/sportmodestate",
                                 self._on_sport, 10)
        self.create_subscription(Float32MultiArray, "/g1_residual/ghost_motion_state",
                                 self._on_ghost, 10)
        self.create_subscription(Float32MultiArray, "/g1_residual/hlc_debug/residual_action",
                                 self._on_residual_action, 10)
        self.create_subscription(PoseStamped, "/g1_residual/hlc_debug/goal_pose_anchor",
                                 self._on_goal_pose_anchor, 10)
        self.create_subscription(Float32MultiArray, "/object_goal",
                                 self._on_goal, 10)

        self.create_timer(1.0 / 50.0, self._tick_scene)

        self.get_logger().info(
            f"viser up on http://localhost:{port}  (urdf joints={self._n_urdf}, "
            f"missing in MJ={(self._urdf_to_mj < 0).sum()}, "
            f"missing in IL={(self._urdf_to_il < 0).sum()})"
        )

    # ── ROS callbacks ───────────────────────────────────────────────
    def _on_lowstate(self, msg: HGLowState):
        # G1 LowState carries 35 motor slots; first 29 are the body in MJ order.
        q = np.array([m.q for m in msg.motor_state[:NQ]], dtype=np.float64)
        quat = np.array(msg.imu_state.quaternion, dtype=np.float64)  # wxyz
        with self._lock:
            self._real_joint_pos_mj = q
            self._real_imu_quat = quat

    def _on_sport(self, msg: SportModeState):
        with self._lock:
            self._real_world_pos = np.array(msg.position, dtype=np.float64)

    def _on_ghost(self, msg: Float32MultiArray):
        d = np.asarray(msg.data, dtype=np.float64)
        if d.size != NQ + 3 + 4:
            return
        now = time.monotonic()
        with self._lock:
            self._ghost_joint_pos_il = d[:NQ].copy()
            self._ghost_anchor_pos   = d[NQ:NQ + 3].copy()
            self._ghost_anchor_quat  = d[NQ + 3:NQ + 7].copy()
            self._ghost_last_t = now

    def _on_goal(self, msg: Float32MultiArray):
        d = np.asarray(msg.data, dtype=np.float64)
        if d.size < 7:
            return
        with self._lock:
            self._goal_pos  = d[0:3].copy()
            self._goal_quat = d[3:7].copy()

    def _on_residual_action(self, msg: Float32MultiArray):
        d = np.asarray(msg.data, dtype=np.float64)
        if d.size != NQ:
            return
        now = time.monotonic()
        with self._lock:
            self._residual_term_mj = d.copy()
            self._residual_last_t = now

    def _on_goal_pose_anchor(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        now = time.monotonic()
        with self._lock:
            self._hlc_goal_pos_b  = np.array([p.x, p.y, p.z], dtype=np.float64)
            # ROS xyzw -> our wxyz
            self._hlc_goal_quat_b = np.array([o.w, o.x, o.y, o.z], dtype=np.float64)
            self._hlc_last_t = now

    # ── viser scene tick (50 Hz) ────────────────────────────────────
    def _tick_scene(self):
        now = time.monotonic()
        with self._lock:
            real_q     = None if self._real_joint_pos_mj is None else self._real_joint_pos_mj.copy()
            real_quat  = None if self._real_imu_quat is None else self._real_imu_quat.copy()
            real_pos   = self._real_world_pos.copy()
            ghost_q    = None if self._ghost_joint_pos_il is None else self._ghost_joint_pos_il.copy()
            ghost_pos  = None if self._ghost_anchor_pos is None else self._ghost_anchor_pos.copy()
            ghost_quat = None if self._ghost_anchor_quat is None else self._ghost_anchor_quat.copy()
            res_mj     = None if self._residual_term_mj is None else self._residual_term_mj.copy()
            ghost_age    = now - self._ghost_last_t    if self._ghost_last_t    > 0 else float("inf")
            residual_age = now - self._residual_last_t if self._residual_last_t > 0 else float("inf")
            hlc_age      = now - self._hlc_last_t      if self._hlc_last_t      > 0 else float("inf")
            gpos       = None if self._goal_pos  is None else self._goal_pos.copy()
            gquat      = None if self._goal_quat is None else self._goal_quat.copy()
            hlc_p      = None if self._hlc_goal_pos_b  is None else self._hlc_goal_pos_b.copy()
            hlc_q      = None if self._hlc_goal_quat_b is None else self._hlc_goal_quat_b.copy()

        ghost_stale    = ghost_age    > STALE_AFTER
        residual_stale = residual_age > STALE_AFTER
        hlc_stale      = hlc_age      > STALE_AFTER

        # ── reality (white) ──────────────────────────────────────
        if real_q is not None:
            cfg = np.zeros(self._n_urdf, dtype=np.float32)
            mask = self._urdf_to_mj >= 0
            cfg[mask] = real_q[self._urdf_to_mj[mask]]
            self._real_urdf.update_cfg(cfg)
        if real_quat is not None:
            self._real_root.wxyz = real_quat.astype(np.float32)
            self._anchor_root.wxyz = real_quat.astype(np.float32)
        if real_pos is not None:
            self._real_root.position = real_pos.astype(np.float32)
            self._anchor_root.position = real_pos.astype(np.float32)

        # ── demo (green) ─────────────────────────────────────────
        if ghost_q is not None:
            cfg = np.zeros(self._n_urdf, dtype=np.float32)
            mask = self._urdf_to_il >= 0
            cfg[mask] = ghost_q[self._urdf_to_il[mask]]
            self._ghost_urdf.update_cfg(cfg)
        if ghost_stale or ghost_pos is None:
            # park deep underground; non-motion mode shouldn't show a half-buried demo
            self._ghost_root.position = (0.0, 0.0, PARK_Z)
        else:
            self._ghost_root.position = ghost_pos.astype(np.float32)
            if ghost_quat is not None:
                self._ghost_root.wxyz = ghost_quat.astype(np.float32)

        # ── residual overlay (blue) ──────────────────────────────
        # joints = real_q + residual_term, anchored at real root pose.
        # Both vectors are MJ order so we can sum directly.
        if real_q is not None and res_mj is not None and not residual_stale:
            res_q = real_q + res_mj
            cfg = np.zeros(self._n_urdf, dtype=np.float32)
            mask = self._urdf_to_mj >= 0
            cfg[mask] = res_q[self._urdf_to_mj[mask]]
            self._residual_urdf.update_cfg(cfg)
            self._residual_root.position = real_pos.astype(np.float32)
            if real_quat is not None:
                self._residual_root.wxyz = real_quat.astype(np.float32)
        else:
            self._residual_root.position = (0.0, 0.0, PARK_Z)

        # ── object goal in world frame ───────────────────────────
        if gpos is not None:
            self._goal_root.position = gpos.astype(np.float32)
        if gquat is not None:
            self._goal_root.wxyz = gquat.astype(np.float32)

        # ── object goal in anchor frame (HLC's view, lifted back to world) ─
        # obs slot is consistent, this overlay matches /object_goal in world.        
        if hlc_stale:
            self._hlc_goal_root.position = (0.0, 0.0, PARK_Z)
        else:
            # if real_quat is not None:
                
            if hlc_p is not None:
                self._hlc_goal_root.position = hlc_p.astype(np.float32)
            if hlc_q is not None:
                self._hlc_goal_root.wxyz = hlc_q.astype(np.float32)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--urdf", required=True, help="path to G1 URDF (yourdfpy-loadable)")
    p.add_argument("--port", type=int, default=8080)
    args = p.parse_args()

    rclpy.init()
    node = ViserGhostG1(args.urdf, args.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
