#!/usr/bin/env python3
"""Talk to the G1's onboard motion_switcher service, and see who owns /lowcmd.

    source unitree_ros2/setup.sh robot
    python scripts/robot_mode.py check      # which onboard mode is running
    python scripts/robot_mode.py who        # who publishes /lowcmd right now
    python scripts/robot_mode.py release    # hand the motors to low-level control
    python scripts/robot_mode.py select ai  # give them back to the onboard mode

Why this exists
---------------
A G1 boots into an onboard motion mode ("ai", "normal", ...). That mode is a
DDS application on the robot and it *publishes LowCmd on `rt/lowcmd` at 500 Hz*
— the same topic cpp_control publishes on. Two writers, one motor board, and
the onboard one is the one that keeps the robot standing, so it wins.

Nothing about this looks like a failure from the ROS side. `/lowstate` streams,
`ros2 topic hz /lowcmd` is healthy, the controller's own commands go out with a
valid CRC, and the robot ignores them. It is the single difference between
sim2sim (where nothing else is publishing) and hardware, and it is invisible
unless you go looking for the second publisher — `who` does exactly that.

    $ python scripts/robot_mode.py who
    /lowcmd publishers: 2
      _CREATED_BY_BARE_DDS_APP_   <- onboard; release the motion mode
      _CREATED_BY_BARE_DDS_APP_

`release` is not a query. It stops the onboard controller, and a robot that was
standing under it will fall. Suspend it from the gantry, or have it sitting, and
be ready on the remote's damping combo. It refuses to run without `--yes`.
"""

import argparse
import json
import random
import sys
import time

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response

# unitree_sdk2: include/unitree/robot/b2/motion_switcher/motion_switcher_api.hpp
API_CHECK_MODE = 1001
API_SELECT_MODE = 1002
API_RELEASE_MODE = 1003
API_SET_SILENT = 1004
API_GET_SILENT = 1005

SERVICE = "/api/motion_switcher"


class MotionSwitcher(Node):
    def __init__(self):
        super().__init__("motion_switcher_cli")
        self._responses = []
        self.create_subscription(
            Response, f"{SERVICE}/response", self._responses.append, 10
        )
        self._pub = self.create_publisher(Request, f"{SERVICE}/request", 10)

    def settle(self, seconds=1.5):
        """Let discovery match before doing anything.

        The robot's endpoints are bare CycloneDDS writers and readers, not ROS
        nodes, and a request published into an unmatched writer is simply
        dropped -- there is no queue for it to wait in. Publishing immediately
        after construction therefore reports a timeout against a service that
        would have answered instantly.
        """
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, api_id, parameter="", timeout=6.0):
        self.settle()
        self._responses.clear()

        req = Request()
        req.header.identity.id = random.randint(1, 2**31 - 1)
        req.header.identity.api_id = api_id
        req.parameter = parameter

        # Re-sent once a second rather than published once and waited on. These
        # are idempotent queries and the cost of a duplicate is nothing, while
        # the cost of losing the single copy to a late match is a wrong answer
        # about whether the robot is under onboard control.
        deadline = time.time() + timeout
        next_send = 0.0
        while time.time() < deadline:
            if time.time() >= next_send:
                self._pub.publish(req)
                next_send = time.time() + 1.0
            rclpy.spin_once(self, timeout_sec=0.1)
            for msg in self._responses:
                if msg.header.identity.api_id == api_id:
                    return msg
        return None


def _report(msg, what):
    if msg is None:
        print(f"{what}: no response from motion_switcher.")
        print("  Is the robot on this interface?  ros2 topic hz /lowstate")
        return None
    if msg.header.status.code != 0:
        print(f"{what}: robot returned status {msg.header.status.code}")
        return None
    return msg.data


def cmd_check(node, _args):
    data = _report(node.call(API_CHECK_MODE), "check")
    if data is None:
        return 1
    try:
        mode = json.loads(data) if data else {}
    except json.JSONDecodeError:
        print(f"mode: {data!r}")
        return 0
    name = mode.get("name", "")
    if name:
        print(f"onboard motion mode: {name!r} (form {mode.get('form', '?')})")
        print("  It owns the motors and publishes its own LowCmd.")
        print("  Low-level control will be ignored until:  robot_mode.py release --yes")
    else:
        print("onboard motion mode: released -- the motors are free for low-level control.")
    return 0


def cmd_release(node, args):
    if not args.yes:
        print("release stops the onboard controller. A robot standing under it WILL FALL.")
        print("Suspend it or sit it down first, then re-run with --yes.")
        return 2
    if _report(node.call(API_RELEASE_MODE), "release") is None:
        return 1
    print("released.")
    time.sleep(0.5)
    return cmd_check(node, args)


def cmd_select(node, args):
    param = json.dumps({"name": args.name})
    if _report(node.call(API_SELECT_MODE, param), "select") is None:
        return 1
    print(f"selected {args.name!r}.")
    time.sleep(0.5)
    return cmd_check(node, args)


def cmd_who(node, _args):
    node.settle()
    # Ask the graph directly rather than `ros2 topic info`: the ros2 daemon
    # caches participants for minutes after a process is gone, which in this
    # particular investigation is the difference between "the onboard controller
    # is fighting me" and "it exited ten minutes ago".
    for topic in ("/lowcmd", "/lowstate"):
        infos = node.get_publishers_info_by_topic(topic)
        print(f"{topic} publishers: {len(infos)}")
        for info in infos:
            print(f"  {info.node_name or '<bare dds>'}")
        if topic == "/lowcmd" and len(infos) > 1:
            print("  more than one writer on /lowcmd -- the onboard motion mode is")
            print("  almost certainly one of them. robot_mode.py check")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check", help="report the running onboard motion mode")
    sub.add_parser("who", help="list the publishers on /lowcmd and /lowstate")
    rel = sub.add_parser("release", help="stop the onboard mode (THE ROBOT WILL GO LIMP)")
    rel.add_argument("--yes", action="store_true", help="confirm; required")
    sel = sub.add_parser("select", help="start an onboard mode by name, e.g. ai")
    sel.add_argument("name")
    args = ap.parse_args()

    rclpy.init()
    node = MotionSwitcher()
    try:
        rc = {
            "check": cmd_check,
            "who": cmd_who,
            "release": cmd_release,
            "select": cmd_select,
        }[args.cmd](node, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
