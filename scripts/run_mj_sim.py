#!/usr/bin/env python3
"""mj_sim for the difftrack rig, with a camera that follows the robot.

Same simulator, same ROS node, same loop as `ros2 run mj_sim main` -- this
subclasses mj_sim's own `MujocoSimBase` and reuses its `RosNode` rather than
reimplementing anything. What it adds is the camera, real-time pacing, and a
video recorder on F9.

IT ALSO RUNS IN REAL TIME, which upstream does not. mj_sim's loop is paced by
`RosNode`'s rclpy rate at **1000 Hz** while the G1 model's physics timestep is
**2 ms (500 Hz)**, so one wall second buys up to two seconds of simulation. It
does not quite keep up, and what you actually get is 1.4x-1.8x real time, moving
with machine load -- measured here at 70-88 control Hz against a 50 Hz policy
across fourteen runs. Nothing about the physics is wrong (`state_decimation: 10`
still hands the policy exactly 20 ms of physics per control step, which is why
the tracking numbers are valid), but the playback is fast, and a 1.03 s gait
cycle shown at 1.75x is 3.4 steps/s -- a jog, from a clip that walks at 1.94
steps/s. Here the loop is paced against the wall clock instead, and the achieved
factor is printed so it is a measurement rather than a hope.

Use `--speed 0` to free-run (upstream behaviour) when sweeping for numbers
rather than watching, or `--speed 0.5` for half speed to look at a footfall.

WHY IT EXISTS. `mujoco.viewer.launch_passive` opens on MuJoCo's free camera,
which is planted at the world origin looking down at 45 degrees and stays there.
A G1 walking at 1 m/s is out of frame in about four seconds, so every sim2sim
past the first stride is watched from behind, and the natural conclusion --
"the robot fell over" -- is usually wrong. Upstream mj_sim has no hook for this
(the viewer is created inside `MujocoSimBase.__init__`), and mj_sim is a
separate repository, so the camera lives here instead.

HOW IT FOLLOWS. The same way crl-humanoid-ros's monitor does it
(`MuJoCoMonitorApp::updateCameraFollow`): azimuth, elevation and distance are
left alone -- they are yours to drag with the mouse -- and only `lookat` is
moved onto the robot's base each rendered frame. Dragging therefore still works
while following, which is the whole point of doing it this way rather than with
a `trackbodyid` camera the mouse cannot orbit freely.

The one addition is smoothing. A walking G1's pelvis bobs several centimetres
per step and yaws with every stride; feeding that straight into `lookat` shakes
the whole world. `camera.smooth` is a first-order lag on the lookat point, in
seconds, and 0.2 takes the bob out without the frame visibly lagging the robot.

    SIM_ASSETS_PATH=/tmp/difftrack_scene \\
        python3 scripts/run_mj_sim.py --cfgpath /tmp/difftrack_scene/G1.yml

IT ALSO RECORDS. `F9` starts and stops an mp4 of the scene, written to
`cpp_control/recordings/`, the same way crl-humanoid-ros's monitor does it: the
scene is rendered a second time into MuJoCo's offscreen framebuffer and piped
into ffmpeg, so the video is independent of the window size and of whatever is
sitting on top of the window. By default it is clocked on SIMULATION time, so
the file plays at 1x real time even when the simulator was free-running. See
`scripts/frame_recorder.py` for the settings and the environment overrides.
`--record` starts one at t=0 without touching the keyboard, which is what
`run_difftrack_sim2sim.sh -R` uses to come back with a file per run.

Viewer keys: `space` pause -- `V` camera follow on/off -- `F9` record on/off --
`Y` robot onto the ground (clears the bring-up weld) -- `E` frames -- `[` `]`
hanging height.

Optional `camera:` block in the sim yaml, shown with its defaults:

    camera:
      follow: true          # move lookat onto the robot every frame
      body: pelvis          # which body to follow
      azimuth: 120.0        # degrees; 0 is behind a robot facing +X
      elevation: -15.0      # degrees; MuJoCo's own default is -45, i.e. steep
      distance: 3.0         # metres from the lookat point
      height_offset: 0.15   # metres above the base, to frame the torso
      smooth: 0.2           # seconds of lag on the lookat point; 0 = none

an optional `recording:` block (F9), shown with its defaults:

    recording:
      dir: <cpp_control>/recordings
      size: 1280x720        # video resolution, independent of the window
      fps: 30
      clock: sim            # sim = always 1x playback; wall = what you watched
      encoder: libx264      # try h264_nvenc
      crf: 18               # quality, lower is better
      prefix: mj_sim        # <prefix>_<timestamp>.mp4

and a top-level `realtime:` key (same meaning as `--speed`, which overrides it):

    realtime: 1.0           # 1.0 = wall clock, 0.5 = half speed, 0 = free-run
"""

import argparse
import os
import sys
import threading
import time

import mujoco
import numpy as np
import rclpy
import yaml

import mj_sim
from mj_sim.src.ros_node_base import RosNode

from frame_recorder import FrameRecorder, camera_snapshot


# GLFW key code, which is what the passive viewer hands the key callback. F9 is
# not one of the keys MuJoCo's own UI consumes (it takes F1-F5), so it arrives
# here intact. `chr()` of it is a letter-like codepoint, hence the numeric test.
GLFW_KEY_F9 = 298

CAMERA_DEFAULTS = {
    "follow": True,
    "body": "pelvis",
    "azimuth": 120.0,
    "elevation": -15.0,
    "distance": 3.0,
    "height_offset": 0.15,
    "smooth": 0.2,
}


class FollowCamSim(mj_sim.MujocoSimBase):
    """MujocoSimBase whose viewer camera stays on the robot."""

    def __init__(self, *args, camera=None, recording=None, **kwargs):
        # Set BEFORE super().__init__: it launches the viewer, which binds
        # self.viewer_key_callback, and that override reads these.
        self._cam_cfg = dict(CAMERA_DEFAULTS)
        self._cam_cfg.update(camera or {})
        self._follow = bool(self._cam_cfg["follow"])
        self._lookat = None
        self._recorder = None
        self._record_toggle = threading.Event()

        super().__init__(*args, **kwargs)

        # The recorder needs a GL context of its own, which costs ~0.1s and a
        # framebuffer, so it is constructed here but only builds that on the
        # first F9. A run that never records pays nothing.
        if not self.headless:
            self._recorder = FrameRecorder(self.model, recording)

        self._follow_body_id = -1
        if not self.headless:
            name = str(self._cam_cfg["body"])
            self._follow_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if self._follow_body_id < 0:
                print(f"[camera] no body named '{name}'; camera follow disabled")
                self._follow = False
            self._init_camera()

    def _init_camera(self):
        # Every write to viewer.cam is under the handle's lock: the passive
        # viewer renders on its own thread and reads the camera while it does.
        with self.viewer.lock():
            self._set_camera()

    def _set_camera(self):
        cam = self.viewer.cam
        # Free camera, explicitly. A model that ships a <camera> would otherwise
        # decide the framing for us, and then none of the numbers below apply.
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.fixedcamid = -1
        cam.azimuth = float(self._cam_cfg["azimuth"])
        cam.elevation = float(self._cam_cfg["elevation"])
        cam.distance = float(self._cam_cfg["distance"])
        self._lookat = np.array(self._target_lookat(), dtype=float)
        cam.lookat[:] = self._lookat
        print(f"[camera] follow={'on' if self._follow else 'off'} "
              f"body={self._cam_cfg['body']} azimuth={cam.azimuth:.0f} "
              f"elevation={cam.elevation:.0f} distance={cam.distance:.1f}m  "
              f"(V toggles follow; drag to orbit)")

    def _target_lookat(self):
        if self._follow_body_id < 0:
            return [0.0, 0.0, 0.8]
        p = self.data.xpos[self._follow_body_id]
        return [p[0], p[1], p[2] + float(self._cam_cfg["height_offset"])]

    def _update_camera(self):
        if not self._follow or self._follow_body_id < 0:
            return
        target = np.asarray(self._target_lookat(), dtype=float)
        # First-order lag in SECONDS, evaluated per rendered frame, so the feel
        # does not change when viewer_fps does.
        tau = float(self._cam_cfg["smooth"])
        dt = self.viewer_sync_rate
        a = 1.0 if tau <= 1e-6 else min(1.0, dt / tau)
        self._lookat += a * (target - self._lookat)
        with self.viewer.lock():
            self.viewer.cam.lookat[:] = self._lookat

    def reset(self):
        # mj_resetData + mj_forward run in here, so this is the first moment the
        # body positions are real. Re-seed rather than let the camera glide in
        # from the world origin on the first stride.
        super().reset()
        if not self.headless and self._follow_body_id >= 0:
            self._lookat = np.array(self._target_lookat(), dtype=float)
            with self.viewer.lock():
                self.viewer.cam.lookat[:] = self._lookat

    def viewer_key_callback(self, keycode):
        # 'V' and F9 are ours; everything else is mj_sim's (space, E, Y, the
        # bracket keys).
        if keycode == GLFW_KEY_F9:
            # DO NOT touch the recorder here. This callback runs on the VIEWER's
            # UI thread, and the recorder owns an OpenGL context that belongs to
            # the sim-loop thread: making it current from a second thread is an
            # X BadAccess on glXMakeCurrent, which kills the process outright
            # (seen: "Major opcode 149 (GLX), Minor opcode 5"). Hand the request
            # to the loop instead and let it start/stop between steps.
            self._record_toggle.set()
            return
        if chr(keycode) == 'V':
            self._follow = not self._follow
            if self._follow:
                # Re-seed rather than sweeping the camera across the scene from
                # wherever the user left it.
                self._lookat = np.array(self._target_lookat(), dtype=float)
            print("[camera] follow:", "on" if self._follow else "off")
            return
        super().viewer_key_callback(keycode)

    def step_head(self):
        if not self.viewer.is_running():
            # Window closed: finalise the mp4 before the process goes away, or
            # ffmpeg never gets to write the moov atom.
            self.close_recorder()
            sys.exit(0)
        mujoco.mj_step(self.model, self.data)
        self.step_count = (self.step_count + 1) % self.frame_skip
        if self.step_count == 0:
            self._update_camera()
            self.viewer.sync()
        self.tick_recorder()

    def _record_view(self):
        return camera_snapshot(self.viewer)

    def tick_recorder(self):
        """Service the recorder from the sim-loop thread: act on a pending F9,
        then capture a frame if one is due. Called every step rather than every
        viewer sync, because the capture rate is the recorder's own and the call
        costs two float comparisons until a frame is actually due.

        Also called from the PAUSED branch of the main loop, which never reaches
        step_head: a sim-clocked recording has nothing due there (data.time is
        frozen, so the pause is cut out of the video), while a wall-clocked one
        keeps emitting and the pause shows up as the freeze it was."""
        if self._recorder is None:
            return
        if self._record_toggle.is_set():
            self._record_toggle.clear()
            self._recorder.toggle(self.data)
        self._recorder.tick(self.data, self._record_view)

    def start_recording(self):
        if self._recorder is not None:
            self._recorder.start(self.data)

    def close_recorder(self):
        if self._recorder is not None:
            self._recorder.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cfgpath", required=True, help="mj_sim yaml config")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--speed", type=float, default=None,
                    help="wall-clock speed: 1.0 real time (default), 0.5 half, "
                         "0 free-run as fast as the machine allows")
    ap.add_argument("--record", action="store_true",
                    help="start recording immediately, as if F9 were pressed "
                         "at t=0; F9 still stops and restarts it")
    args = ap.parse_args()

    assets = os.environ.get("SIM_ASSETS_PATH")
    if not assets:
        sys.exit("SIM_ASSETS_PATH is not set (source unitree_ros2/setup.sh)")

    cfg = yaml.safe_load(open(args.cfgpath))
    cfg["sim"]["headless"] = args.headless
    cfg["sim"]["model_path"] = os.path.join(assets, cfg["sim"]["model_path"])
    camera = cfg.pop("camera", {})
    recording = cfg.pop("recording", {})
    speed = args.speed if args.speed is not None else float(cfg.pop("realtime", 1.0))

    rclpy.init()
    sim = FollowCamSim(camera=camera, recording=recording, **cfg["sim"])

    robot_cfg = dict(cfg["robot"])
    robot = getattr(mj_sim, robot_cfg.pop("class_name"))(sim, **robot_cfg)
    ros_node = RosNode(robot)

    threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True).start()

    dt = float(sim.model.opt.timestep)
    if speed > 0.0:
        print(f"[rate] physics {1.0 / dt:.0f} Hz, paced to {speed:g}x real time "
              f"({dt / speed * 1e3:.2f} ms of wall clock per step)", flush=True)
    else:
        print(f"[rate] physics {1.0 / dt:.0f} Hz, FREE-RUNNING — the viewer will "
              f"play faster than real time", flush=True)

    sim.reset()
    robot_state = robot.data2state()

    # After the reset, so the recording does not open on the pre-reset pose --
    # and so a sim-clocked recording starts from data.time = 0.
    if args.record:
        sim.start_recording()

    step_wall = dt / speed if speed > 0.0 else 0.0
    next_step = time.perf_counter() + step_wall
    t_report = time.perf_counter()
    steps_since_report = 0

    try:
        while True:
            if sim.viewer_pause:
                # Do NOT publish while paused. The controller ticks its control loop
                # off arriving state messages (`state_decimation`), so republishing a
                # frozen state runs the policy forward against a robot that is not
                # moving: the clip walks away from a standing robot and the run is
                # spoiled by the time you unpause. Withholding the stream stops the
                # control loop instead, and the >5*control_dt gap on resume makes the
                # node re-anchor and restart the clip, which is what "pause" should
                # have meant.
                sim.tick_recorder()
                time.sleep(0.01)
                next_step = time.perf_counter() + step_wall
                t_report = time.perf_counter()
                steps_since_report = 0
                continue

            # mj_sim's own limp fallback lives in here: with no publisher on
            # /robot_command it overwrites every motor command with kp=0 kd=0.1.
            ros_node.check_topic_publishers('/robot_command')
            robot.command2ctrl(ros_node.robot_command)
            sim.step()
            robot_state = robot.data2state()
            ros_node.publish_robot_state(robot_state)

            if speed <= 0.0:
                ros_node.rate.sleep()          # upstream: an rclpy rate, not the clock
                continue

            # Pace against an ABSOLUTE schedule, not by sleeping a fixed slice: sleep
            # overshoots by tens of microseconds every call, and a relative pacer
            # accumulates that into a steady rate error. Here an overshoot just eats
            # the next slack and the mean rate stays right.
            now = time.perf_counter()
            slack = next_step - now
            if slack > 0:
                time.sleep(slack)
            elif slack < -0.25:
                next_step = now                # fell far behind; do not sprint to catch up
            next_step += step_wall

            steps_since_report += 1
            if now - t_report >= 10.0:
                achieved = steps_since_report * dt / (now - t_report)
                print(f"[rate] {achieved:.2f}x real time "
                      f"({steps_since_report / (now - t_report):.0f} Hz physics, "
                      f"{steps_since_report / (now - t_report) / 10:.1f} Hz control)",
                      flush=True)
                t_report, steps_since_report = now, 0
    finally:
        # Ctrl-C, a closed window or a controller that exited: ffmpeg still
        # has to be told the stream is over, or the mp4 has no index and no
        # player will open it.
        sim.close_recorder()


if __name__ == "__main__":
    main()
