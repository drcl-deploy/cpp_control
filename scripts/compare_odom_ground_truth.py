#!/usr/bin/env python3
"""Score the onboard state estimator against the simulator's ground truth.

    python3 scripts/compare_odom_ground_truth.py [-d SECONDS] [--csv OUT.csv]

Subscribes to both and reports how far apart they are:

    /odom             nav_msgs/Odometry     the estimator (docker/estimator)
    /sportmodestate   SportModeState        unitree_mujoco's ground-truth
                                            position and world linear velocity
    /lowstate         unitree_hg/LowState   ground-truth pelvis attitude and
                                            body-frame gyro

WHY THIS EXISTS, AND WHY IT IS THE FIRST THING TO RUN

The estimator is the one part of this deployment whose failure mode is a
plausible number. A wrong twist frame, a gravity sign, a joint permutation --
none of them raise, none of them stop the robot standing, and all of them show
up on hardware as "the policy transferred worse than it did in sim". The
container can be up, publishing at 500 Hz, and completely wrong.

So before letting the estimator drive:

    bash scripts/run_difftrack_sim2sim.sh -E compare -d 15 g1_walk

which runs the controller on GROUND TRUTH -- a known-good run -- with the
estimator alongside, and calls this script on the result.

WHAT THE NUMBERS MEAN

    pos_drift     |estimated - true| position, metres. GROWS: there is no
                  absolute position reference in the loop, so x and y are dead
                  reckoning. What matters is the drift accumulated over ONE
                  CLIP, which is why the report gives it per second as well.
    height_err    the z component alone, and it should NOT grow -- foot contact
                  makes height observable. A growing height error means the
                  contact measurement is not being applied: wrong foot frames,
                  a force threshold the robot never crosses, or a joint map that
                  puts the legs somewhere else.
    lin_vel_err   world-frame linear velocity, m/s. The observation term the
                  policy is most sensitive to, and the one a wrong twist frame
                  ruins. A frame error shows up here as an error that TRACKS
                  THE HEADING: near zero while the robot faces +x, large once it
                  turns. The report gives the correlation with heading for
                  exactly that reason.
    yaw_err       heading, radians. Drifts, like position.

A gravity-handling error is unmissable rather than subtle: lin_vel_err runs
away at ~9.8 m/s per second and pos_drift goes quadratic. If you see that, the
accelerometer convention is wrong, not the tuning.
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry

try:
    from unitree_go.msg import SportModeState
    from unitree_hg.msg import LowState
except ImportError as exc:  # pragma: no cover
    print(f"needs the unitree message packages on AMENT_PREFIX_PATH: {exc}", file=sys.stderr)
    print("  source unitree_ros2/setup.sh", file=sys.stderr)
    raise SystemExit(2)


def quat_to_yaw(w, x, y, z):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Comparator(Node):
    def __init__(self, duration, csv_path):
        super().__init__("difftrack_odom_compare")
        self.duration = duration
        self.rows = []
        self.csv_path = csv_path

        self.est = None          # (pos, quat, lin_world)
        self.true_pos = None
        self.true_vel = None
        self.true_quat = None

        # The estimator publishes best-effort; a RELIABLE subscription would
        # never match it and this script would report "no samples" rather than
        # a comparison.
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(SportModeState, "/sportmodestate", self.on_sport, 10)
        self.create_subscription(LowState, "/lowstate", self.on_low, qos_profile_sensor_data)
        self.t0 = None

    # ---- inputs ----------------------------------------------------------
    def on_sport(self, msg):
        self.true_pos = (msg.position[0], msg.position[1], msg.position[2])
        self.true_vel = (msg.velocity[0], msg.velocity[1], msg.velocity[2])

    def on_low(self, msg):
        q = msg.imu_state.quaternion
        self.true_quat = (q[0], q[1], q[2], q[3])

    def on_odom(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t = msg.twist.twist.linear

        # The estimator publishes a CHILD-frame twist (REP-105). Rotate it into
        # the world here so the comparison is against a world-frame ground
        # truth -- doing it the other way round would hide a frame error behind
        # a second one.
        qw, qx, qy, qz = o.w, o.x, o.y, o.z
        vb = (t.x, t.y, t.z)
        vw = rotate_by_quat(qw, qx, qy, qz, vb)
        self.est = ((p.x, p.y, p.z), (qw, qx, qy, qz), vw)

        if self.true_pos is None or self.true_quat is None:
            return
        now = time.time()
        if self.t0 is None:
            self.t0 = now
        self.rows.append(
            (
                now - self.t0,
                self.est[0],
                self.true_pos,
                self.est[2],
                self.true_vel or (0.0, 0.0, 0.0),
                quat_to_yaw(qw, qx, qy, qz),
                quat_to_yaw(*self.true_quat),
            )
        )


def rotate_by_quat(w, x, y, z, v):
    """v_world = q * v_body * q^-1, written out to avoid a numpy dependency."""
    vx, vy, vz = v
    # t = 2 * (q_vec x v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def norm(a, b):
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def report(rows, csv_path):
    if len(rows) < 10:
        print(f"\ncompare: only {len(rows)} paired samples — is the estimator running?")
        print("         docker ps | grep estimator      # the container")
        print("         ros2 topic hz /odom             # is it publishing")
        return 1

    span = rows[-1][0] - rows[0][0]
    pos_err = [norm(r[1], r[2]) for r in rows]
    h_err = [abs(r[1][2] - r[2][2]) for r in rows]
    v_err = [norm(r[3], r[4]) for r in rows]
    yaw_err = [abs(wrap(r[5] - r[6])) for r in rows]

    def stats(xs):
        s = sorted(xs)
        return (
            sum(xs) / len(xs),
            s[int(0.95 * (len(s) - 1))],
            max(xs),
        )

    print(f"\n  {len(rows)} paired samples over {span:.1f} s\n")
    print(f"  {'':14s} {'mean':>9s} {'p95':>9s} {'max':>9s} {'final':>9s}")
    for name, xs, fin in (
        ("pos_drift  m", pos_err, pos_err[-1]),
        ("height_err m", h_err, h_err[-1]),
        ("lin_vel_err", v_err, v_err[-1]),
        ("yaw_err  rad", yaw_err, yaw_err[-1]),
    ):
        m, p95, mx = stats(xs)
        print(f"  {name:14s} {m:9.4f} {p95:9.4f} {mx:9.4f} {fin:9.4f}")

    rate = pos_err[-1] / span if span > 0 else 0.0
    print(f"\n  position drift rate   {rate:.4f} m/s of wall clock")

    # Height is the tell for the contact measurement, and it is the one that
    # should NOT grow: a foot on the ground pins it. Compare the first and last
    # thirds rather than the endpoints, which are noisy.
    third = max(1, len(h_err) // 3)
    early = sum(h_err[:third]) / third
    late = sum(h_err[-third:]) / third
    print(f"  height error          {early:.4f} m early -> {late:.4f} m late", end="")
    if late > max(0.05, 3.0 * early):
        print("   <-- GROWING: the contact update is not being applied")
        print("      check the foot frame names, the joint map, and the contact force threshold")
    else:
        print("   (bounded, as it should be — feet are pinning it)")

    # A twist-frame error is a velocity error that appears only once the robot
    # has turned away from +x. Correlating the two separates it from noise.
    turned = [r for r in rows if abs(wrap(r[6])) > 0.3]
    if len(turned) > 10:
        vt = sum(norm(r[3], r[4]) for r in turned) / len(turned)
        straight = [r for r in rows if abs(wrap(r[6])) <= 0.3]
        vs = (sum(norm(r[3], r[4]) for r in straight) / len(straight)) if straight else float("nan")
        print(f"  lin_vel_err vs heading  {vs:.4f} facing +x  ->  {vt:.4f} turned", end="")
        if vt > max(0.15, 3.0 * vs):
            print("   <-- FRAME ERROR")
            print("      the twist is being read in the wrong frame; check odom_twist_frame")
        else:
            print("   (no heading dependence — the frame is right)")
    else:
        print("  lin_vel_err vs heading  the robot never turned far enough to test the frame")

    if csv_path:
        with open(csv_path, "w") as f:
            f.write("t,ex,ey,ez,tx,ty,tz,evx,evy,evz,tvx,tvy,tvz,eyaw,tyaw\n")
            for r in rows:
                f.write(
                    "%.4f,%s,%s,%s,%s,%.5f,%.5f\n"
                    % (
                        r[0],
                        ",".join("%.5f" % v for v in r[1]),
                        ",".join("%.5f" % v for v in r[2]),
                        ",".join("%.5f" % v for v in r[3]),
                        ",".join("%.5f" % v for v in r[4]),
                        r[5],
                        r[6],
                    )
                )
        print(f"\n  csv: {csv_path}")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-d", "--duration", type=float, default=15.0)
    ap.add_argument("--csv", default="")
    args = ap.parse_args()

    rclpy.init()
    node = Comparator(args.duration, args.csv)
    print(f"compare: collecting for {args.duration:.0f} s …")
    end = time.time() + args.duration
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    rc = report(node.rows, args.csv)
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
