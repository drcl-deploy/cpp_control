#!/usr/bin/env python3
"""Record a difftrack policy's ACTIONS off the wire, one row per control step.

    python3 scripts/record_policy_actions.py --config <export>/difftrack_config.json \
                                             --out run.npz [--idle-timeout 6]

Run it BESIDE a sim2sim session; scripts/compare_action_smoothness.sh does that
for you. It publishes nothing and subscribes to two topics, so it cannot change
the run it is measuring -- which is the whole reason the actions are recovered
from /lowcmd instead of being logged inside the controller.

WHY /lowcmd IS ENOUGH

The difftrack node turns an action into a joint target with one line
(DiffTrackObsBuilder::actionToJointTarget) and then into a LowCmd with one more
(G1DiffTrackNode::command_from_target):

    q*[m] = default_angles[p] + action_scale * a[p]        m = policy_to_motor[p]

action_scale and default_angles are in the export's difftrack_config.json, the
same file the node reads, so the action is recoverable EXACTLY -- no clipping is
in the way unless the export asks for it (action_clipping, off on every shipped
policy, and reported here when it is on). Nothing is inferred and nothing is
resampled: the control loop publishes exactly one LowCmd per control step
(BaseNode::control_loop), so one message is one policy inference.

WHICH SAMPLES ARE THE POLICY'S

Every other control mode -- zeroing, damping, the nominal pose, the entry ramp,
the rest state -- also publishes a LowCmd, and its q looks like a joint target
too. The GAINS are what separates them. The policy drives through the
checkpoint's own trained stiffness (joint_stiffness in the export, 14-99 Nm/rad
on the shipped G1 policies); every hold mode drives through the yaml's much
stiffer hold gains (300-400), and the entry/exit ramps interpolate between the
two. So a sample is the policy's exactly when kp equals joint_stiffness, and
that test is exact rather than a threshold because both numbers come from the
same float32s in the same file.

This file stores kp and kd unmodified and lets the analysis decide; it only uses
the test to report a window and to know when the run has actually started.

WHAT COMES OUT

An npz with the raw capture (nothing derived):

    t_cmd      [N]      seconds, monotonic, since the first LowCmd
    q_cmd      [N, 29]  commanded joint target, MOTOR order
    kp, kd     [N, 29]  the gains that came with it
    t_state    [M]      the same clock, for the LowState stream
    q_meas     [M, 29]  measured joint position, motor order
    dq_meas    [M, 29]  measured joint velocity
    tau_meas   [M, 29]  measured joint torque
    ref_dof    [K, 29]  the reference clip's own joint angles, POLICY order,
                        read out of motion.bin -- the "signal" the actions are
                        supposed to be tracking, and the floor any smoothness
                        number should be read against
    meta       json     export config, joint orders, train run, control_dt

Dropped messages are not silently smoothed over: t_cmd is kept so the analysis
can find a gap and refuse to difference across it.
"""

import argparse
import json
import os
import signal
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data

try:
    from unitree_hg.msg import LowCmd, LowState
except ImportError as exc:  # pragma: no cover
    print(f"needs the unitree message packages on AMENT_PREFIX_PATH: {exc}", file=sys.stderr)
    print("  source unitree_ros2/setup.sh sim", file=sys.stderr)
    raise SystemExit(2)


# The unitree_hg motor order for the 29-dof G1, which is also the MuJoCo DFS
# order and what every g1_difftrack_*.yaml lists as `joint_names`. It is fixed
# by the hg IDL, so it is a constant here rather than a parameter -- but
# --robot-config overrides it from a yaml for a robot whose map differs.
G1_MOTOR_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]


def read_ref_dof(cfg, config_path):
    """The clip's own joint angles out of motion.bin.

    The file is a flat little-endian float32 dump in the order
    DiffTrackObsBuilder::loadMotion reads it; every array's shape is already in
    the config under motion_bin, so the offset of the last one is arithmetic
    rather than a guess. A mismatch means the export is inconsistent with its
    own motion file and is reported instead of being read past.
    """
    mb = cfg.get("motion_bin")
    if not mb:
        return None
    path = os.path.join(os.path.dirname(config_path), cfg["motion_name"])
    order = ["tar_root_pos", "tar_root_quat", "tar_joint_rot6", "tar_key_rel",
             "ref_root_pos", "ref_root_quat", "ref_dof_pos"]
    counts = [int(np.prod(mb[k])) for k in order]
    total = sum(counts)
    want = total * 4
    have = os.path.getsize(path)
    if have != want:
        print(f"  motion.bin is {have} bytes, the config describes {want} — not reading it")
        return None
    offset = sum(counts[:-1]) * 4
    n = counts[-1]
    with open(path, "rb") as f:
        f.seek(offset)
        buf = f.read(n * 4)
    return np.frombuffer(buf, dtype="<f4", count=n).reshape(mb["ref_dof_pos"])


class ActionRecorder(Node):
    def __init__(self, args, cfg):
        super().__init__("difftrack_action_recorder")
        self.args = args
        self.cfg = cfg
        self.nm = len(args.motor_names)

        # RELIABLE, and deep. The controller publishes /lowcmd on the default
        # rclcpp profile (reliable, depth 10); a best-effort subscription would
        # match it and then quietly drop samples under load, which is the one
        # failure that would make a policy look JITTERIER than it is -- a
        # dropped step doubles the next first difference.
        cmd_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=400)
        self.create_subscription(LowCmd, args.lowcmd_topic, self.on_cmd, cmd_qos)
        # LowState comes from the simulator's DDS bridge best-effort, so this
        # one has to be sensor-data or it never matches.
        self.create_subscription(LowState, args.lowstate_topic, self.on_state,
                                 qos_profile_sensor_data)

        self.t_cmd, self.q_cmd, self.kp, self.kd = [], [], [], []
        self.t_state, self.q_meas, self.dq_meas, self.tau_meas = [], [], [], []
        self.t0 = None
        self.t_boot = time.monotonic()
        self.last_cmd_t = None
        self.stop = False
        self.policy_seen = 0

        # kp as it will look while the policy is driving, in MOTOR order.
        self.kp_policy = np.zeros(self.nm, dtype=np.float32)
        for p, m in enumerate(args.policy_to_motor):
            self.kp_policy[m] = cfg["joint_stiffness"][p]

        self.create_timer(0.25, self.tick)
        self.get_logger().info(
            f"recording {args.lowcmd_topic} + {args.lowstate_topic} -> {args.out}")

    # ---- inputs ----------------------------------------------------------
    def now(self):
        t = time.monotonic()
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    def on_cmd(self, msg):
        t = self.now()
        mc = msg.motor_cmd
        self.t_cmd.append(t)
        self.q_cmd.append([mc[i].q for i in range(self.nm)])
        kp = [mc[i].kp for i in range(self.nm)]
        self.kp.append(kp)
        self.kd.append([mc[i].kd for i in range(self.nm)])
        self.last_cmd_t = time.monotonic()
        if np.allclose(kp, self.kp_policy, rtol=1e-4, atol=1e-4):
            self.policy_seen += 1

    def on_state(self, msg):
        t = self.now()
        ms = msg.motor_state
        self.t_state.append(t)
        self.q_meas.append([ms[i].q for i in range(self.nm)])
        self.dq_meas.append([ms[i].dq for i in range(self.nm)])
        self.tau_meas.append([ms[i].tau_est for i in range(self.nm)])

    # ---- lifetime --------------------------------------------------------
    def tick(self):
        if self.stop:
            raise SystemExit(0)
        if self.t0 is not None and (time.monotonic() - self.t0) > self.args.max_seconds:
            self.get_logger().warn(f"hit --max-seconds {self.args.max_seconds}; stopping")
            self.stop = True
        # A run that never starts -- a build that is not there, a simulator
        # that will not open -- must not leave this holding a terminal until
        # --max-seconds. It has heard nothing at all, so there is nothing to
        # save and nothing to wait for.
        if self.last_cmd_t is None and \
                (time.monotonic() - self.t_boot) > self.args.start_timeout:
            self.get_logger().error(
                f"nothing published {self.args.lowcmd_topic} in "
                f"{self.args.start_timeout:.0f}s — no controller ever started")
            self.stop = True
        # The controller exiting is the normal end of a measured run, and it
        # takes /lowcmd with it. Waiting for the topic to go quiet is what makes
        # this script self-terminating rather than something the driver has to
        # remember to kill at the right moment.
        if self.last_cmd_t is not None and \
                (time.monotonic() - self.last_cmd_t) > self.args.idle_timeout:
            self.get_logger().info(
                f"no {self.args.lowcmd_topic} for {self.args.idle_timeout:.1f}s — the run is over")
            self.stop = True

    def save(self):
        n = len(self.t_cmd)
        if n == 0:
            print(f"NO SAMPLES on {self.args.lowcmd_topic} — nothing was controlling a robot.",
                  file=sys.stderr)
            return 1
        meta = {
            "motion": self.args.motion,
            "config_path": os.path.abspath(self.args.config),
            "train_run": self.cfg.get("train_run", ""),
            "clip_name": self.cfg.get("clip_name", ""),
            "source_run": self.cfg.get("source_run", ""),
            "checkpoint_iter": self.cfg.get("checkpoint_iter", -1),
            "control_dt": self.cfg["control_dt"],
            "action_scale": self.cfg["action_scale"],
            "action_clipping": self.cfg.get("action_clipping", False),
            "action_clip_range": self.cfg.get("action_clip_range", 0.0),
            "default_angles": self.cfg["default_angles"],
            "joint_stiffness": self.cfg["joint_stiffness"],
            "joint_damping": self.cfg["joint_damping"],
            "policy_joint_names": self.cfg["policy_joint_names"],
            "motor_joint_names": self.args.motor_names,
            "policy_to_motor": self.args.policy_to_motor,
            "clip_steps": self.cfg["obs"]["clip_steps"],
            "loop_wrap": self.cfg["obs"]["loop_wrap"],
            "motion_length_s": self.cfg["obs"]["motion_length_s"],
            "label": self.args.label or self.args.motion,
            "note": self.args.note,
        }
        arrays = dict(
            t_cmd=np.asarray(self.t_cmd, dtype=np.float64),
            q_cmd=np.asarray(self.q_cmd, dtype=np.float32),
            kp=np.asarray(self.kp, dtype=np.float32),
            kd=np.asarray(self.kd, dtype=np.float32),
            t_state=np.asarray(self.t_state, dtype=np.float64),
            q_meas=np.asarray(self.q_meas, dtype=np.float32),
            dq_meas=np.asarray(self.dq_meas, dtype=np.float32),
            tau_meas=np.asarray(self.tau_meas, dtype=np.float32),
            meta=np.array(json.dumps(meta)),
        )
        ref = read_ref_dof(self.cfg, self.args.config)
        if ref is not None:
            arrays["ref_dof"] = np.ascontiguousarray(ref)
        os.makedirs(os.path.dirname(os.path.abspath(self.args.out)) or ".", exist_ok=True)
        np.savez_compressed(self.args.out, **arrays)
        print(f"{self.args.out}: {n} commands ({self.policy_seen} with the policy's gains), "
              f"{len(self.t_state)} states")
        if self.policy_seen == 0:
            print("  NONE of them were the policy's — the run never left its hold mode.",
                  file=sys.stderr)
            return 1
        return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", required=True,
                    help="the export's difftrack_config.json")
    ap.add_argument("--out", required=True, help="output .npz")
    ap.add_argument("--motion", default="", help="the export directory's name, for the label")
    ap.add_argument("--label", default="", help="what to call this run in the plots")
    ap.add_argument("--note", default="", help="free text carried into the npz")
    ap.add_argument("--lowcmd-topic", default="/lowcmd")
    ap.add_argument("--lowstate-topic", default="/lowstate")
    ap.add_argument("--robot-config", default="",
                    help="a g1_difftrack_*.yaml, if the motor order is not the stock G1 one")
    ap.add_argument("--start-timeout", type=float, default=240.0,
                    help="give up if no /lowcmd ever arrives")
    ap.add_argument("--idle-timeout", type=float, default=6.0,
                    help="stop this many seconds after /lowcmd goes quiet")
    ap.add_argument("--max-seconds", type=float, default=900.0)
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = json.load(f)
    if not args.motion:
        args.motion = os.path.basename(os.path.dirname(os.path.abspath(args.config)))

    motor_names = list(G1_MOTOR_JOINTS)
    if args.robot_config:
        import yaml
        with open(args.robot_config) as f:
            motor_names = yaml.safe_load(f)["joint_names"]
    args.motor_names = motor_names

    # By NAME, exactly as G1DiffTrackNode does it at startup. An unplaceable
    # name is fatal there and fatal here: a silent fallback to index order is
    # how a permutation becomes a smoothness result.
    try:
        args.policy_to_motor = [motor_names.index(n) for n in cfg["policy_joint_names"]]
    except ValueError as exc:
        print(f"a policy joint is not in the motor order: {exc}", file=sys.stderr)
        return 2

    rclpy.init()
    node = ActionRecorder(args, cfg)

    def on_signal(_sig, _frm):
        node.stop = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        while rclpy.ok() and not node.stop:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, SystemExit):
        pass
    rc = node.save()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
