#!/usr/bin/env python3
"""Watch the observe block decide: no staging, no clicks, no bag."""

# The calibration viewer exists to LABEL frames; this one exists to answer "is
# the palette right, here, now". It shows the frame, what the classifier made of
# every pixel, and the verdict the planner would believe — including the
# candidates the gates threw away, because a wrong colour and a rejected face
# fail differently.
#
#     ros2 run cpp_control repose_check_observe.py

import argparse

from cpp_control.msg import CubeObservation
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CompressedImage, Image

NAMES = ('red', 'orange', 'green', 'yellow', 'blue', 'pink')
# BGR, matching CubeSight::paint so the overlay and the label plane agree.
SWATCH = ((51, 51, 230), (26, 115, 242), (51, 230, 51),
          (51, 230, 230), (230, 51, 51), (166, 51, 204))


def qos():
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


class Check(Node):

    def __init__(self, args):
        super().__init__('repose_check_observe')
        import cv2
        self.cv2 = cv2
        self.args = args
        self.color = self.labels = None
        self.obs = None
        self.create_subscription(CompressedImage, args.color_topic, self._color, qos())
        self.create_subscription(Image, args.labels_topic, self._labels, qos())
        self.create_subscription(CubeObservation, args.observation_topic,
                                 self._obs, qos())
        self.create_timer(1.0 / args.fps, self._draw)
        self.get_logger().info(f'watching {args.observation_topic} — q or Esc to quit')

    def _color(self, m):
        self.color = self.cv2.imdecode(np.frombuffer(m.data, np.uint8),
                                       self.cv2.IMREAD_COLOR)

    def _labels(self, m):
        self.labels = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)

    def _obs(self, m):
        self.obs = m

    def _paint(self, labels):
        out = np.full((*labels.shape, 3), 30, np.uint8)
        for c in range(6):
            out[labels == c] = SWATCH[c]
        return out

    def _draw(self):
        cv2 = self.cv2
        if self.color is None:
            return
        h = self.args.height
        w = int(self.color.shape[1] * h / self.color.shape[0])
        left = cv2.resize(self.color, (w, h), interpolation=cv2.INTER_AREA)
        right = (cv2.resize(self._paint(self.labels), (w, h),
                            interpolation=cv2.INTER_NEAREST)
                 if self.labels is not None else np.zeros_like(left))
        canvas = np.hstack([left, right])
        pad = np.full((150, canvas.shape[1], 3), 20, np.uint8)
        canvas = np.vstack([canvas, pad])
        y = h + 26

        def line(text, colour=(230, 230, 230), dy=22):
            nonlocal y
            cv2.putText(canvas, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        colour, 1, cv2.LINE_AA)
            y += dy

        o = self.obs
        if o is None:
            line('no observation yet — is the planner observe node up?', (60, 160, 240))
        elif not o.state_ready:
            line('NO LOWSTATE: without the IMU and waist there is no base frame,',
                 (60, 160, 240))
            line('so every geometric gate below would be judging an invented pose.',
                 (60, 160, 240))
        else:
            if o.color_ok and 0 <= o.color < 6:
                line(f'{NAMES[o.color].upper():<8} {o.reason}', SWATCH[o.color], 26)
            else:
                line(f'{"REJECTED":<8} {o.reason}', (60, 160, 240), 26)
            line(f'pose_ok {int(o.pose_ok)}   range {o.range_m:5.2f} m   '
                 f'bearing {np.degrees(o.bearing_rad):+6.1f} deg   '
                 f'phi {np.degrees(o.phi_rad):+5.1f} deg   n_px {o.n_px}')
            # The candidates are the whole diagnosis: one face split across two
            # chromaticity cells shows up here as two rows and nowhere else.
            parts = []
            for i, c in enumerate(o.candidate_color):
                nm = NAMES[c] if 0 <= c < 6 else '?'
                parts.append(f'{nm}:{o.candidate_pixels[i]}px '
                             f'up{o.candidate_up_dot[i]:.2f} '
                             f'vis{o.candidate_visible[i]:.2f} '
                             f'big{o.candidate_big[i]:.2f}')
            line('candidates: ' + ('  |  '.join(parts) if parts else 'none'),
                 (170, 170, 170))
            if not o.stand_locked:
                line('stand not locked — the controller is not holding the nominal stance',
                     (60, 160, 240))

        cv2.imshow('repose observe', canvas)
        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--color-topic', default='/vibe/planner/observe/color/compressed')
    ap.add_argument('--labels-topic', default='/vibe/planner/observe/labels')
    ap.add_argument('--observation-topic', default='/vibe/planner/observe/observation')
    ap.add_argument('--height', type=int, default=420)
    ap.add_argument('--fps', type=float, default=20.0)
    args = ap.parse_args()

    rclpy.init()
    node = Check(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
