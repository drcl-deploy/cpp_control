#!/usr/bin/env python3
"""Terminal attention viewer — who's looking at which patch, live.

Subscribes to the adapt-sonic attention mask (queries x patches, softmax rows)
and redraws a braille-intensity table at ~10 Hz:

    q_task_cmd |    ⡀ ⣄ ⣿ ⣶ ⡄        ⡀
    q_proprio  | ⡀ ⡄ ⣄ ⣤ ⣤ ⣄ ⡄ ⡀
    q_cls      |          ⣿ ⣷ ⡄

Cells are row-normalized (each row's max = full glyph). Query names come from
the msg layout label — nothing hardcoded.

Usage:
    ros2 run cpp_control attn_viewer.py
    python3 scripts/attn_viewer.py --topic /vibe/sonic/attention_mask
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

RAMP = ' ⡀⡄⣄⣤⣦⣶⣷⣿'  # 9 intensity levels, dots -> dense cluster


class AttnViewer(Node):
    def __init__(self, topic):
        super().__init__('attn_viewer')
        self.msg = None
        self.count = 0
        self.t0 = time.monotonic()
        self.create_subscription(Float32MultiArray, topic, self._on_msg, 10)
        self.topic = topic

    def _on_msg(self, msg):
        self.msg = msg
        self.count += 1

    def draw(self):
        sys.stdout.write('\x1b[H\x1b[J')  # cursor home + clear
        hz = self.count / max(time.monotonic() - self.t0, 1e-6)
        if self.msg is None:
            print(f'attn_viewer: waiting for {self.topic} ...')
            sys.stdout.flush()
            return

        rows = self.msg.layout.dim[0].size
        cols = self.msg.layout.dim[1].size
        names = (self.msg.layout.dim[0].label or '').split('|')
        if len(names) != rows:
            names = [f'q{i}' for i in range(rows)]
        width = max(len(n) for n in names)

        print(f'{self.topic}   {rows} queries x {cols} patches   {hz:5.1f} Hz')
        print(f'{"":{width}} | ' + ' '.join(f'{i % 10}' for i in range(cols)))
        print('-' * (width + 3 + 2 * cols))
        for r, name in enumerate(names):
            row = self.msg.data[r * cols:(r + 1) * cols]
            m = max(row) or 1.0
            cells = ' '.join(RAMP[round(v / m * (len(RAMP) - 1))] for v in row)
            print(f'{name:{width}} | {cells}   (max {m:.2f} @ patch {row.index(max(row))})')
        sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/vibe/sonic/attention_mask')
    args = parser.parse_args()

    rclpy.init()
    viewer = AttnViewer(args.topic)
    last_draw = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(viewer, timeout_sec=0.1)
            if time.monotonic() - last_draw > 0.1:  # redraw at ~10 Hz, not msg rate
                viewer.draw()
                last_draw = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
