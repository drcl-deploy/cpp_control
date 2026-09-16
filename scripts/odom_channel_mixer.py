#!/usr/bin/env python3
"""Feed the controller a world state assembled channel by channel from ground
truth or the onboard estimator -- and log both while it runs.

    python3 scripts/odom_channel_mixer.py --pos-xy est --pos-z gt \
        --vel-xy gt --vel-z gt [--out /odom_mixed] [--csv OUT.csv]

Run by `run_difftrack_sim2sim.sh -E mixed` (ODOM_MIX picks the channels); see
docs/trackers/difftrack_state_estimation.md.

WHY THIS EXISTS

`-E ground_truth` and `-E estimator` differ in four observation channels at
once, and a policy that survives one and falls on the other says nothing about
WHICH. The difftrack observation reads exactly four things out of the world
state that the estimator does not get from the IMU:

    pos_xy   the robot's position under the reference, in every tar_obs frame
             (reference root xy minus robot root xy)
    pos_z    the absolute root height in char_obs
    vel_xy   horizontal world linear velocity in char_obs
    vel_z    vertical world linear velocity in char_obs

Orientation and angular velocity are the IMU's in both modes (the estimator
does not filter attitude), so they are not a variable and come from LowState
here. Swapping one channel at a time to the estimator, with the rest held at
ground truth, is how the failure is attributed.

INPUTS / OUTPUT

    /sportmodestate  SportModeState  unitree_mujoco ground truth: position and
                                     WORLD linear velocity (what -E
                                     ground_truth feeds the controller)
    /lowstate        LowState        IMU quaternion + body gyro
    /odom            Odometry        docker/estimator, REP-105 (child twist)
    --out            Odometry        pose in world, twist ALREADY IN WORLD --
                                     launch the controller with
                                     odom_twist_frame:=world

One message goes out per SportModeState, i.e. at the simulator's state rate,
carrying the freshest sample of each source. With every channel on `gt` the
controller sees exactly what -E ground_truth hands it, one DDS hop later --
the control run that shows this node itself costs nothing.

THE CSV

One row per published message: time, then ground truth and estimate for
position and world velocity side by side, so the estimator's error can be read
along the trajectory the policy actually produced -- including a run the
estimator is driving, which -E compare cannot show. Flushed every 100 rows, so
a SIGKILL from the rig's cleanup loses at most a fifth of a second.

    t, gx,gy,gz, ex,ey,ez, gvx,gvy,gvz, evx,evy,evz, est_age
"""

import argparse
import math
import signal
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
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

CHANNELS = ("pos_xy", "pos_z", "vel_xy", "vel_z")


def rotate(w, x, y, z, v):
    """v_world = q * v_body * q^-1."""
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


class Mixer(Node):
    def __init__(self, src, out_topic, csv_path, csv_every=4):
        super().__init__("difftrack_odom_channel_mixer")
        self.src = src
        self.csv_every = max(1, int(csv_every))
        self.need_est = any(v == "est" for v in src.values())

        self.imu_quat = None      # (w, x, y, z)
        self.gyro = None          # body frame
        self.est = None           # (pos, world vel, receive time)

        self.pub = self.create_publisher(Odometry, out_topic, qos_profile_sensor_data)
        self.create_subscription(SportModeState, "/sportmodestate", self.on_sport, 10)
        self.create_subscription(LowState, "/lowstate", self.on_low, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)

        self.csv = open(csv_path, "w") if csv_path else None
        if self.csv:
            self.csv.write("t,gx,gy,gz,ex,ey,ez,gvx,gvy,gvz,evx,evy,evz,est_age\n")
            self.csv.flush()
        self.rows = 0
        self.t0 = None
        self.published = 0
        self.received = {"lowstate": 0, "sportmodestate": 0, "odom": 0}
        self.ticks = 0
        self.create_timer(0.5, self.on_timer)

        spec = " ".join(f"{k}={v}" for k, v in src.items())
        self.get_logger().info(f"odom mixer: {spec} -> {out_topic} (twist in world frame)")

    def on_timer(self):
        # Flush on a clock as well as every 100 rows, so a run that ends in the
        # rig's SIGKILL keeps its tail -- and while nothing is going out, say
        # which input is missing, because the controller only reports that it
        # is still waiting for a world state.
        if self.csv:
            self.csv.flush()
        self.ticks += 1
        if self.ticks % 4 == 0 and (self.published == 0 or self.ticks <= 8):
            r = self.received
            self.get_logger().info(
                "received lowstate=%d sportmodestate=%d odom=%d, published %d"
                % (r["lowstate"], r["sportmodestate"], r["odom"], self.published))

    def on_low(self, msg):
        self.received["lowstate"] += 1
        # Plain floats throughout: the unitree arrays hold numpy float32, and
        # the geometry_msgs converters reject those with an ASSERTION -- a core
        # dump, not an exception -- the moment one reaches an outgoing message.
        q = msg.imu_state.quaternion
        self.imu_quat = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        self.gyro = tuple(float(v) for v in msg.imu_state.gyroscope)

    def on_odom(self, msg):
        self.received["odom"] += 1
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t = msg.twist.twist.linear
        # REP-105: the estimator's twist is in the child (body) frame.
        vw = rotate(o.w, o.x, o.y, o.z, (t.x, t.y, t.z))
        self.est = ((p.x, p.y, p.z), vw, time.monotonic())

    def on_sport(self, msg):
        self.received["sportmodestate"] += 1
        if self.imu_quat is None:
            return
        if self.need_est and self.est is None:
            return
        now = time.monotonic()
        gp = tuple(float(v) for v in msg.position[:3])
        gv = tuple(float(v) for v in msg.velocity[:3])
        if self.est is not None:
            ep, ev, erx = self.est
            age = now - erx
        else:
            ep, ev, age = (math.nan,) * 3, (math.nan,) * 3, math.nan

        def pick(ch, g, e):
            return e if self.src[ch] == "est" else g

        pos = (*pick("pos_xy", gp[:2], ep[:2]), pick("pos_z", gp[2], ep[2]))
        vel = (*pick("vel_xy", gv[:2], ev[:2]), pick("vel_z", gv[2], ev[2]))

        w, x, y, z = self.imu_quat
        ang = rotate(w, x, y, z, self.gyro)

        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "odom"
        out.child_frame_id = "pelvis"
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = pos
        out.pose.pose.orientation.w = w
        out.pose.pose.orientation.x = x
        out.pose.pose.orientation.y = y
        out.pose.pose.orientation.z = z
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = vel
        out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = ang
        self.pub.publish(out)
        self.published += 1

        if self.csv and self.published % self.csv_every == 0:
            if self.t0 is None:
                self.t0 = now
            vals = (now - self.t0, *gp, *ep, *gv, *ev, age)
            self.csv.write(",".join("%.5f" % v for v in vals) + "\n")
            self.rows += 1
            if self.rows % 100 == 0:
                self.csv.flush()

    def close(self):
        if self.csv:
            self.csv.close()
            self.csv = None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    for ch in CHANNELS:
        ap.add_argument("--" + ch.replace("_", "-"), choices=("gt", "est"), default="gt")
    ap.add_argument("--out", default="/odom_mixed")
    ap.add_argument("--csv", default="")
    ap.add_argument("--csv-every", type=int, default=4,
                    help="log every Nth published sample (default 4: ~250 Hz of a 1 kHz state stream)")
    args = ap.parse_args()

    src = {ch: getattr(args, ch) for ch in CHANNELS}
    rclpy.init()
    node = Mixer(src, args.out, args.csv, args.csv_every)

    # The rig stops this with SIGTERM. Shutting the context down from the
    # handler interrupts whatever take() the executor is in, and that surfaces
    # as a RuntimeError out of spin -- expected here, and only here.
    stopping = []

    def stop(*_):
        stopping.append(True)
        node.close()
        rclpy.try_shutdown()

    signal.signal(signal.SIGTERM, stop)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if not stopping:
            raise
    finally:
        node.close()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
