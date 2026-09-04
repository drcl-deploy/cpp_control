#!/usr/bin/env python3
"""Can the hold gains hold this robot up? One question, one answer, no policy.

The difftrack node's hardest moment is not tracking, it is the second before it:
a joint-space PD ramping the robot from wherever it is into the policy's nominal
pose and holding it there. Everything else is downstream of that working. This
runs exactly that, and nothing else -- no ONNX, no clip, no observation builder,
no controller -- against a running `unitree_mujoco`, and says HELD or FELL.

    # terminal 1
    source unitree_ros2/setup.sh
    env -u LD_LIBRARY_PATH $UNITREE_MUJOCO/simulate/build/unitree_mujoco \\
        -r g1 -t 1 -c -i $ROS_DOMAIN_ID -n lo -s <scene>

    # terminal 2
    source unitree_ros2/setup.sh
    python3 scripts/probe_hold_gains.py --config <export>/difftrack_config.json

Options that matter, because they are what the answer turns on:

    --kp LEG KNEE ANKLE WAIST ARM   default 300 400 300 300 60 (the shipped hold)
    --kd-ratio R                    kd = kp/R. THE knob -- see below.
    --hold-measured                 hold the pose it is already in instead of
                                    ramping to the nominal one. Separates "can a
                                    PD hold this robot" from "can it get there".

WHAT THIS FOUND, and why it is worth keeping around:

`kd = kp/20`, which g1_difftrack.yaml and g1_difftrack_hw.yaml both carry,
does not hold a G1 on unitree_mujoco. Twelve leg joints pin at FULL torque with
the sign alternating between samples while the position errors are still around
0.06 rad -- an oscillating loop, not a balance failure -- and the robot is on
the floor in a second. kp/30 and below are clean.

It never showed up on mj_sim because mj_sim rewrites MuJoCo's actuator
gainprm/biasprm: there the PD is a POSITION SERVO inside the solver and its
damping is integrated implicitly. unitree_mujoco computes the torque itself and
writes it to ctrl -- explicit damping, on a thread that is not even synchronous
with the physics. A real motor controller is the second kind.

So: if you change the hold gains, or take them to a robot, run this first.
Read the `#lim` column, not just the verdict -- joints at full torque with
alternating signs is the signature, and it is visible long before the fall.
"""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from unitree_go.msg import SportModeState
from unitree_hg.msg import LowCmd, LowState

N = 29


def parse_args():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True,
                    help="difftrack_config.json from the export; its "
                         "default_angles are the pose being held")
    ap.add_argument("--kp", nargs=5, type=float, metavar=("LEG", "KNEE", "ANKLE",
                                                          "WAIST", "ARM"),
                    default=[300.0, 400.0, 300.0, 300.0, 60.0])
    ap.add_argument("--kd-ratio", type=float, default=40.0,
                    help="kd = kp / this (default 40, what the unitree config "
                         "ships; 20 is what mj_sim's configs use and it rings)")
    ap.add_argument("--settle", type=float, default=2.0, help="ramp seconds")
    ap.add_argument("--hold", type=float, default=6.0, help="seconds after the ramp")
    ap.add_argument("--hold-measured", action="store_true",
                    help="hold the starting pose instead of the nominal one")
    return ap.parse_args()


class Probe(Node):
    def __init__(self, args, target, kp, kd, limits):
        super().__init__("probe_hold_gains")
        self.a, self.target, self.kp, self.kd, self.lim = args, target, kp, kd, limits
        self.pub = self.create_publisher(LowCmd, "/lowcmd", 10)
        self.create_subscription(LowState, "/lowstate", self._on_low, 10)
        self.create_subscription(SportModeState, "/sportmodestate", self._on_sport, 10)
        self.q = self.tau = self.z = self.t0 = self.q0 = None
        self.rows = []
        self.create_timer(0.02, self._tick)

    def _on_low(self, m):
        self.q = [m.motor_state[i].q for i in range(N)]
        self.tau = [m.motor_state[i].tau_est for i in range(N)]

    def _on_sport(self, m):
        self.z = m.position[2]

    def _tick(self):
        if self.q is None or self.z is None:
            return
        if self.t0 is None:
            self.t0, self.q0 = time.time(), list(self.q)
        t = time.time() - self.t0
        alpha = min(1.0, t / self.a.settle) if self.a.settle > 0 else 1.0
        goal = self.q0 if self.a.hold_measured else self.target

        cmd = LowCmd()
        cmd.mode_pr, cmd.mode_machine = 0, 5
        for i in range(N):
            cmd.motor_cmd[i].q = (1 - alpha) * self.q0[i] + alpha * goal[i]
            cmd.motor_cmd[i].kp = self.kp[i]
            cmd.motor_cmd[i].kd = self.kd[i]
        self.pub.publish(cmd)

        if not self.rows or t - self.rows[-1][0] >= 0.5:
            # Joints at (or within 10% of) their torque limit. The count alone
            # is not the tell -- an arm brushing its 25 N.m limit is harmless.
            # Full-scale LEG torque with the sign flipping between rows is.
            hot = [(i, round(self.tau[i], 1)) for i in range(N)
                   if abs(self.tau[i]) > 0.9 * self.lim[i]]
            self.rows.append((t, self.z, hot))
        if t > self.a.settle + self.a.hold:
            raise SystemExit


def main():
    args = parse_args()
    conf = json.load(open(args.config))
    target = conf["default_angles"]
    if len(target) != N:
        sys.exit(f"{args.config} has {len(target)} default_angles, expected {N}")

    leg, knee, ank, waist, arm = args.kp
    kp = [leg, leg, leg, knee, ank, ank] * 2 + [waist] * 3 + [arm] * 14
    kd = [k / args.kd_ratio for k in kp]
    # The G1's own motor limits, in unitree_hg motor order -- the same numbers
    # unitree's g1_29dof.xml puts in ctrlrange, which is what makes saturation
    # here mean saturation on the robot.
    limits = ([88, 88, 88, 139, 50, 50] * 2) + [88, 50, 50] + [25] * 14

    rclpy.init()
    node = Probe(args, target, kp, kd, limits)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass

    if not node.rows:
        print("no state received -- is unitree_mujoco running on this domain?")
        rclpy.shutdown()
        sys.exit(1)

    print(f"kp leg/knee/ankle/waist/arm = {leg:.0f}/{knee:.0f}/{ank:.0f}/"
          f"{waist:.0f}/{arm:.0f}   kd = kp/{args.kd_ratio:g}   "
          f"target = {'measured pose' if args.hold_measured else 'nominal pose'}")
    print("     t(s)   pelvis z   joints at torque limit (motor, N.m)")
    for t, z, hot in node.rows:
        print(f"   {t:6.2f}   {z:8.3f}   {hot if hot else ''}")
    held = node.rows[-1][1] > 0.6
    print("HELD" if held else "FELL")
    rclpy.shutdown()
    sys.exit(0 if held else 1)


if __name__ == "__main__":
    main()
