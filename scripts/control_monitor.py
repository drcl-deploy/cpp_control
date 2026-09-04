#!/usr/bin/env python3
"""Watch gamepad presses and the resulting /lowcmd side by side. Read-only.

    source unitree_ros2/setup.sh robot
    python scripts/control_monitor.py          # press B, Y, then X

Prints a line whenever the button word changes, and whenever the commanded
gains/targets change. That pairing is the point: a press with no following
command change means the controller is not acting on the gamepad; a command
change with no robot motion means the robot is not following the command. They
are different faults with the same symptom, and nothing short of watching both
ends at once separates them.

Also reports, per writer, what is actually on /lowcmd. `_CREATED_BY_BARE_DDS_APP_`
there is the robot's own onboard controller.
"""

import struct
import sys
import time

import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowCmd, LowState

BITS = ["R1", "L1", "start", "select", "R2", "L2", "F1", "F2",
        "A", "B", "X", "Y", "up", "right", "down", "left"]

# What each control mode looks like on the wire, from base.cpp. kp is the
# discriminator: only NOMINAL_POSE / STANDING_UP / POLICY command stiffness.
def classify(kp, kd, q_nonzero):
    if kp == 0 and kd == 0:
        return "ZEROING (limp, zero torque)"
    if kp == 0 and kd > 0:
        return "DAMPING (limp, damped)"
    if kp > 0 and not q_nonzero:
        return "position hold at q=0 (?)"
    return "POSITION control (nominal_pose / policy)"


class Monitor(Node):
    def __init__(self):
        super().__init__("control_monitor")
        self.create_subscription(LowState, "/lowstate", self._on_state, 10)
        self.create_subscription(LowCmd, "/lowcmd", self._on_cmd, 50)
        self.prev_btn = None
        self.prev_sig = None
        self.n_state = 0
        self.n_cmd = 0
        self.q_meas = [0.0] * 29
        self.q_cmd = [0.0] * 29
        self.kp_cmd = [0.0] * 29
        self.next_track = 0.0

    def _stamp(self):
        return time.strftime("%H:%M:%S")

    def _on_state(self, msg):
        self.n_state += 1
        for i in range(29):
            self.q_meas[i] = msg.motor_state[i].q
        (btn,) = struct.unpack_from("<H", bytes(msg.wireless_remote), 2)
        if self.prev_btn is None:
            self.prev_btn = btn
            return
        if btn != self.prev_btn:
            for i, n in enumerate(BITS):
                if btn & (1 << i) and not self.prev_btn & (1 << i):
                    print(f"{self._stamp()}  BUTTON  {n}")
            self.prev_btn = btn

    def _on_cmd(self, msg):
        self.n_cmd += 1
        mc = msg.motor_cmd
        self.q_cmd = [mc[i].q for i in range(29)]
        self.kp_cmd = [mc[i].kp for i in range(29)]
        kp = round(mc[0].kp, 1)
        kd = round(mc[0].kd, 1)
        kp3 = round(mc[3].kp, 1)
        q_nonzero = any(abs(mc[i].q) > 1e-4 for i in range(29))
        sig = (kp, kd, kp3, q_nonzero)
        if sig != self.prev_sig:
            self.prev_sig = sig
            print(f"{self._stamp()}  LOWCMD  kp[0]={kp} kd[0]={kd} kp[knee]={kp3} "
                  f"targets={'set' if q_nonzero else 'all zero'}   -> {classify(kp, kd, q_nonzero)}")

        # Under position control, whether the robot FOLLOWS is the whole answer.
        # Printed once a second so a 2 s ramp is visible without drowning the log.
        if kp > 0 and time.time() >= self.next_track:
            self.next_track = time.time() + 1.0
            err = max(abs(self.q_cmd[i] - self.q_meas[i]) for i in range(29))
            worst = max(range(29), key=lambda i: abs(self.q_cmd[i] - self.q_meas[i]))
            print(f"{self._stamp()}  TRACK   knee cmd={self.q_cmd[3]:+.3f} meas={self.q_meas[3]:+.3f} | "
                  f"worst joint {worst}: cmd={self.q_cmd[worst]:+.3f} meas={self.q_meas[worst]:+.3f} "
                  f"err={err:.3f} rad")

    def report_tracking(self):
        print()
        print(f"{'joint':>12} {'cmd q':>8} {'meas q':>8} {'kp':>7} {'err':>7}")
        for nm, i in [("L_hip_pitch", 0), ("L_knee", 3), ("R_hip_pitch", 6),
                      ("R_knee", 9), ("waist_yaw", 12)]:
            print(f"  {nm:>10} {self.q_cmd[i]:>8.3f} {self.q_meas[i]:>8.3f} "
                  f"{self.kp_cmd[i]:>7.1f} {abs(self.q_cmd[i]-self.q_meas[i]):>7.3f}")
        if max(self.kp_cmd) == 0:
            print()
            print("No position command was ever sent during this run -- every command was")
            print("zero-torque or damping, neither of which can move the robot. Press X.")


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
    rclpy.init()
    node = Monitor()
    print(f"watching for {seconds:.0f}s. Press B, then Y, then X. ^C to stop early.")
    deadline = time.time() + seconds
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    print()
    print(f"/lowstate messages: {node.n_state}    /lowcmd messages: {node.n_cmd}")
    writers = node.get_publishers_info_by_topic("/lowcmd")
    print(f"/lowcmd writers: {len(writers)}")
    for w in writers:
        print(f"  {w.node_name or '<bare dds>'}")
    node.report_tracking()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
