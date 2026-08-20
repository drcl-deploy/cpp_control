#!/usr/bin/env python3
"""the planner's state pane, in a terminal — and the one place you set the target colour.

    ros2 run cpp_control repose_console.py

Keys: 0-5 or r/o/g/y/b/p to pick a colour, R to re-arm on the same one, q to
quit. `done` LATCHES, so R is how a solved cube starts the next trial.
The colour goes out on /vibe/sonic/goal_color, which BOTH the planner and the
policy's object_goal_color one-hot read — one topic, so they cannot disagree
about what "4" means.
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32

from cpp_control.msg import ControllerStatus, PlannerStatus

NAMES = ("red", "orange", "green", "yellow", "blue", "pink")
KEYS = "rogybp"
RGB = ((230, 51, 51), (242, 115, 26), (51, 230, 51),
       (230, 230, 51), (51, 51, 230), (204, 51, 166))
MODES = ("init", "settle", "scan", "clip", "scan-k")
CONTROLLER_MODES = ("zeroing", "damping", "nominal", "standing", "stand", "POLICY")

CSI = "\033["
DIM, BOLD, OFF = f"{CSI}2m", f"{CSI}1m", f"{CSI}0m"


def chip(i: int, text: str = "  ") -> str:
    if not 0 <= i < len(RGB):
        return f"{DIM}--{OFF}"
    r, g, b = RGB[i]
    return f"{CSI}48;2;{r};{g};{b}m{text}{OFF}"


def swatch(i: int) -> str:
    r, g, b = RGB[i]
    return f"{CSI}38;2;{r};{g};{b}m"


class Console(Node):
    def __init__(self, args) -> None:
        super().__init__("console")
        self.planner: PlannerStatus | None = None
        self.controller: ControllerStatus | None = None
        self.selected = args.color
        best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PlannerStatus, args.planner_topic, self._on_planner, 10)
        self.create_subscription(ControllerStatus, args.controller_topic, self._on_controller, best)
        self.pub = self.create_publisher(Int32, args.goal_topic, 10)

    def _on_planner(self, msg: PlannerStatus) -> None:
        self.planner = msg

    def _on_controller(self, msg: ControllerStatus) -> None:
        self.controller = msg

    def set_color(self, i: int) -> None:
        self.selected = i
        self.pub.publish(Int32(data=i))

    # -- rendering: one frame, redrawn in place --

    def render(self) -> str:
        s1, s0 = self.planner, self.controller
        w = 78
        rule = f"{DIM}{'─' * w}{OFF}"
        ver = s1.version if s1 else "?"
        head = f"{BOLD} the planner · repose{OFF}    {DIM}{ver}{OFF}"

        picker = "  ".join(
            (f"{BOLD}{swatch(i)}[{i}]{NAMES[i]}{OFF}" if i == self.selected
             else f"{DIM}[{i}]{NAMES[i]}{OFF}")
            for i in range(6))

        lines = [head, rule, f"  target  {chip(self.selected)}  {picker}"]
        if s1 is None:
            lines += [f"  {DIM}waiting for {self.get_name()} … is the planner up?{OFF}"]
        else:
            d = chr(s1.delta) if 32 <= s1.delta < 127 else "?"
            stall = f"{s1.stall}" if s1.stall else f"{DIM}0{OFF}"
            # The belief is a vote, so show the tally: a decision standing on
            # 3 of 3 reads and one standing on 3 of 12 are not the same claim.
            # Pad before colouring — escape codes are width 0 but len() > 0.
            vote = ("blind" if s1.n_reads == 0
                    else f"{s1.n_votes}/{s1.n_reads} reads").ljust(12)
            if s1.n_reads == 0:
                vote = f"{DIM}{vote}{OFF}"
            # Range/bearing/spin belong to the pose, so blank them without one:
            # a stale row that looks live is worse than an empty one.
            where = (f"r {s1.range_m:.2f} m   bearing {s1.bearing_rad * 57.3:+.0f}°"
                     f"   spin {s1.phi_rad * 57.3:+.0f}°   {s1.n_px} px"
                     if s1.pose_ok else f"{DIM}no pose{OFF}")
            lines += [
                f"  mode    {BOLD}{MODES[s1.mode]:<7}{OFF}{s1.label:<10}"
                f"rung {s1.rung}  d={d}  tips {s1.tips}  stall {stall}  "
                f"burned {s1.burned}   {DIM}roll{OFF} {s1.rolls}  "
                f"{DIM}ep{OFF} {s1.episode}"
                + (f"  {BOLD}DONE{OFF}" if s1.done else ""),
                f"  belief  {chip(s1.color) if s1.sees else f'{DIM}--{OFF}'}  "
                f"{vote}{s1.reason:<10}{where}",
                f"  gate    ω {s1.omega_cam:.2f} rad/s"
                f"   read {s1.accept_rate * 100:.0f}% of frames"
                f"   {s1.observe_ms:.1f} ms/read",
                f"  plan    {s1.label:<10}cost {s1.cost:.2f} m   "
                f"yaw {s1.entry_yaw:+.2f} rad   frames {s1.frames}"
                + (f" (+{s1.lead_in_frames} ramp)" if s1.lead_in_frames else ""),
                f"  {DIM}next{OFF}    {s1.cand_label:<10}cost {s1.cand_cost:.2f} m"
                f"   {DIM}(what a finished reference would commit to){OFF}",
            ]

        lines.append(rule)
        if s0 is None:
            lines.append(f"  ctrl    {DIM}no status — is the controller on planner:=true?{OFF}")
        else:
            mode = CONTROLLER_MODES[s0.control_mode] if s0.control_mode < 6 else "?"
            what = "stand" if s0.stand else (s0.reference_id or "—")
            lines.append(
                f"  ctrl    {BOLD}{mode:<8}{OFF}{what:<14}"
                f"frame {s0.frame}/{s0.frames}"
                f"   {'● engaged' if s0.accepting else f'{DIM}○ idle{OFF}'}")
        lines += [rule,
                  f"  {DIM}0-5 / {'/'.join(KEYS)}  set target     R  re-arm (next trial)"
                  f"     q  quit{OFF}"]
        return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--color", type=int, default=4, help="initial target (0-5)")
    ap.add_argument("--goal-topic", default="/vibe/sonic/goal_color")
    ap.add_argument("--planner-topic", default="/vibe/planner/status")
    ap.add_argument("--controller-topic", default="/vibe/controller/status")
    args = ap.parse_args()

    rclpy.init()
    node = Console(args)
    fd = sys.stdin.fileno()
    interactive = os.isatty(fd)
    saved = termios.tcgetattr(fd) if interactive else None
    if interactive:
        tty.setcbreak(fd)
    print(f"{CSI}?25l", end="")           # hide the cursor
    height = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if interactive and select.select([fd], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key in ("q", "\x03"):
                    break
                if key.isdigit() and 0 <= int(key) < 6:
                    node.set_color(int(key))
                elif key == "R":
                    node.set_color(node.selected)   # same colour = next episode
                elif key.lower() in KEYS:
                    node.set_color(KEYS.index(key.lower()))
            frame = node.render()
            if height:
                print(f"{CSI}{height}A", end="")
            print("\n".join(f"{CSI}2K{ln}" for ln in frame.split("\n")))
            height = frame.count("\n") + 1
    except KeyboardInterrupt:
        pass
    finally:
        print(f"{CSI}?25h", end="", flush=True)
        if interactive:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
