#!/usr/bin/env python3
"""
Offboard viewer for /enc/frame with policy attention overlaid.

Subscribes to ImageTokens only to learn the patch grid. Attention is currently
unstamped, so this is a live diagnostic overlay rather than exact frame sync.

Keys: [ / ] cycle mean and individual queries; q or Esc quits.
"""

import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from vision_encoders.msg import ImageTokens


def _label(image, lines):
    output = image.copy()
    for index, line in enumerate(lines):
        cv2.putText(
            output, line, (8, 24 + index * 24), cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (0, 255, 0), 2, cv2.LINE_AA)
    return output


class AttentionOverlayViewer(Node):
    """Render the latest frame and query-selectable patch attention."""

    def __init__(self, args):
        super().__init__('attention_overlay_viewer')
        self.args = args
        self.frame = None
        self.grid = None
        self.attention = None
        self.query_names = []
        self.selection = -1  # -1 is mean; 0..N-1 are query rows.
        self.should_quit = False
        self.frame_count = 0
        self.attention_count = 0
        self.started = time.monotonic()
        self.warned_encoding = False

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, args.frame_topic, self._on_frame, qos)
        self.create_subscription(ImageTokens, args.tokens_topic, self._on_tokens, qos)
        self.create_subscription(
            Float32MultiArray, args.attention_topic, self._on_attention, qos)
        self.create_timer(1.0 / 30.0, self._draw)

    def _on_frame(self, msg):
        if msg.encoding not in ('bgr8', 'rgb8'):
            if not self.warned_encoding:
                self.get_logger().warning(
                    f'unsupported {self.args.frame_topic} encoding: {msg.encoding}')
                self.warned_encoding = True
            return
        array = np.frombuffer(msg.data, dtype=np.uint8)
        expected = int(msg.height) * int(msg.width) * 3
        if array.size < expected:
            return
        frame = array[:expected].reshape(int(msg.height), int(msg.width), 3).copy()
        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        self.frame = frame
        self.frame_count += 1

    def _on_tokens(self, msg):
        if msg.grid_h and msg.grid_w:
            self.grid = (int(msg.grid_h), int(msg.grid_w))

    def _on_attention(self, msg):
        if len(msg.layout.dim) < 2:
            return
        rows = int(msg.layout.dim[0].size)
        cols = int(msg.layout.dim[1].size)
        if rows <= 0 or cols <= 0 or len(msg.data) < rows * cols:
            return
        self.attention = np.asarray(msg.data[:rows * cols], dtype=np.float32).reshape(rows, cols)
        names = (msg.layout.dim[0].label or '').split('|')
        self.query_names = names if len(names) == rows else [f'q{i}' for i in range(rows)]
        if self.selection >= rows:
            self.selection = -1
        self.attention_count += 1

    def _selected_attention(self):
        if self.selection < 0:
            return self.attention.mean(axis=0), 'mean(all queries)'
        return self.attention[self.selection], self.query_names[self.selection]

    def _draw(self):
        if self.frame is None:
            return

        height = max(1, round(self.args.width * self.frame.shape[0] / self.frame.shape[1]))
        base = cv2.resize(self.frame, (self.args.width, height), interpolation=cv2.INTER_NEAREST)
        elapsed = max(time.monotonic() - self.started, 1e-6)
        rates = (
            f'frame {self.frame_count / elapsed:.1f} Hz | '
            f'attn {self.attention_count / elapsed:.1f} Hz')
        left = _label(base, [self.args.frame_topic, rates])

        status = 'waiting for attention + token grid'
        overlay = base.copy()
        if self.attention is not None and self.grid is not None:
            grid_h, grid_w = self.grid
            values, status = self._selected_attention()
            if values.size == grid_h * grid_w:
                patch_map = values.reshape(grid_h, grid_w)
                maximum = float(np.max(patch_map))
                if maximum > 0.0:
                    patch_map = np.clip(patch_map / maximum, 0.0, 1.0)
                heat = cv2.applyColorMap(
                    np.asarray(patch_map * 255.0, dtype=np.uint8), cv2.COLORMAP_JET)
                heat = cv2.resize(heat, (self.args.width, height), interpolation=cv2.INTER_NEAREST)
                overlay = cv2.addWeighted(base, 1.0 - self.args.alpha, heat, self.args.alpha, 0.0)
                status = f'{status} | grid {grid_h}x{grid_w} | [ ] query'
            else:
                status = f'attention has {values.size} patches; tokens report {grid_h}x{grid_w}'

        right = _label(overlay, [self.args.attention_topic, status])
        cv2.imshow('ViBe Attention Overlay (latest, unstamped)', np.hstack((left, right)))
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            self.should_quit = True
        elif key == ord('[') and self.attention is not None:
            self.selection -= 1
            if self.selection < -1:
                self.selection = self.attention.shape[0] - 1
        elif key == ord(']') and self.attention is not None:
            self.selection += 1
            if self.selection >= self.attention.shape[0]:
                self.selection = -1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frame-topic', default='/enc/frame')
    parser.add_argument('--tokens-topic', default='/enc/tokens')
    parser.add_argument('--attention-topic', default='/vibe/sonic/attention_mask')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--alpha', type=float, default=0.45)
    args, ros_args = parser.parse_known_args()
    if args.width < 64:
        parser.error('--width must be at least 64')
    if not 0.0 <= args.alpha <= 1.0:
        parser.error('--alpha must be between 0 and 1')

    rclpy.init(args=ros_args)
    viewer = AttentionOverlayViewer(args)
    try:
        while rclpy.ok() and not viewer.should_quit:
            rclpy.spin_once(viewer, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
