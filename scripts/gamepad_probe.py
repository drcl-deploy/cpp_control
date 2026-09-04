#!/usr/bin/env python3
"""Decode LowState.wireless_remote and print button transitions. Read-only.

    source unitree_ros2/setup.sh robot
    python scripts/gamepad_probe.py          # press buttons; ^C to stop

Why this exists
---------------
"I press B and nothing happens" has three completely different causes and they
are indistinguishable from the robot's behaviour:

1. the remote's bytes never reach `/lowstate` -- nothing to decode;
2. they do, the controller switches mode, and the ROBOT ignores it because the
   onboard motion mode still owns the motors (robot_mode.py check);
3. the controller is not running, and the `/lowcmd` you are echoing belongs to
   the robot's own onboard controller, which publishes on the same topic.

This settles (1) without the controller running at all, and reports the /lowcmd
writer count so (3) cannot be mistaken for success.

The layout is unitree_sdk2's xRockerBtnDataStruct, byte for byte the same one
include/common/gamepad.hpp casts onto -- so what prints here is exactly what
G1Node::handle_gamepad() sees.
"""

import struct
import sys
import time

import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowState

# Bit order of xKeySwitchUnion, LSB first (include/common/gamepad.hpp).
BITS = ["R1", "L1", "start", "select", "R2", "L2", "F1", "F2",
        "A", "B", "X", "Y", "up", "right", "down", "left"]


def decode(buf):
    head = buf[0:2]
    (btn,) = struct.unpack_from("<H", buf, 2)
    lx, rx, ry, l2, ly = struct.unpack_from("<5f", buf, 4)
    names = [n for i, n in enumerate(BITS) if btn & (1 << i)]
    return head, btn, names, (lx, rx, ry, l2, ly)


class Probe(Node):
    def __init__(self):
        super().__init__("gamepad_probe")
        self.create_subscription(LowState, "/lowstate", self._on_state, 10)
        self.prev = None
        self.count = 0
        self.seen_any_button = False

    def _on_state(self, msg):
        buf = bytes(msg.wireless_remote)
        self.count += 1
        head, btn, names, sticks = decode(buf)

        if self.prev is None:
            print(f"first LowState: head={head.hex()} btn=0x{btn:04x} "
                  f"sticks lx={sticks[0]:+.2f} ly={sticks[4]:+.2f} "
                  f"rx={sticks[1]:+.2f} ry={sticks[2]:+.2f} L2={sticks[3]:+.2f}")
            if head != b"\x55\x51":
                print(f"  WARNING head is {head.hex()}, expected 5551 -- this is not"
                      " the layout gamepad.hpp decodes.")
            self.prev = btn
            return

        if btn != self.prev:
            self.seen_any_button = True
            pressed = [n for i, n in enumerate(BITS)
                       if btn & (1 << i) and not self.prev & (1 << i)]
            released = [n for i, n in enumerate(BITS)
                        if self.prev & (1 << i) and not btn & (1 << i)]
            stamp = time.strftime("%H:%M:%S")
            for n in pressed:
                print(f"{stamp}  PRESS   {n}")
            for n in released:
                print(f"{stamp}  release {n}")
            self.prev = btn


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.init()
    node = Probe()

    print(f"listening on /lowstate for {seconds:.0f}s -- press B, then Y, then A.")
    deadline = time.time() + seconds
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    print()
    print(f"LowState messages: {node.count}")
    if node.count == 0:
        print("  Nothing arrived. Wrong interface, or the robot is not streaming:")
        print("    source unitree_ros2/setup.sh robot ; ros2 topic hz /lowstate")
    elif not node.seen_any_button:
        print("  State streamed, but the button word NEVER changed.")
        print("  The remote is not reaching wireless_remote -- check it is powered")
        print("  and paired. No controller-side setting can fix this.")
    else:
        print("  Buttons ARE reaching /lowstate, so G1Node::handle_gamepad() sees them.")
        print("  If the robot still does not react, the onboard motion mode owns the")
        print("  motors: scripts/robot_mode.py check")

    writers = len(node.get_publishers_info_by_topic("/lowcmd"))
    print(f"/lowcmd publishers: {writers}")
    if writers:
        print("  NOTE: the robot's onboard controller publishes on /lowcmd too, so a")
        print("  non-empty `ros2 topic echo /lowcmd` does NOT mean your node is running.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
