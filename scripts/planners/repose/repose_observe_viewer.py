#!/usr/bin/env python3
"""
Run the offboard RGB-D viewer and snapshot labeler for The planner color calibration.

An onboard gamepad LT edge (or Space) selects the next complete paired RGB-D
frame. Selected frames are republished on reliable snapshot topics, and their
CalibrationState carries the exact source sequence and ground-truth label.

Keys: Space click, p cancel queued click, r retry current stage, q/Esc quit.
"""

import argparse
from collections import OrderedDict
import time

from cpp_control.msg import CalibrationState, CubeObservation
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import UInt64


COLOR_NAMES = ('red', 'orange', 'green', 'yellow', 'blue', 'pink')
STAGE_NAMES = COLOR_NAMES + ('negative / no cube',)
LABEL_BGR = np.asarray([
    (51, 51, 230), (26, 115, 242), (51, 230, 51),
    (51, 230, 230), (230, 51, 51), (166, 51, 204),
], dtype=np.uint8)


def _stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def _lines(image, lines, color=(0, 255, 0), scale=0.62):
    output = image.copy()
    for index, line in enumerate(lines):
        cv2.putText(output, line, (10, 26 + 25 * index),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)
    return output


class ObserveViewer(Node):
    """Pair, display, and label the live The planner observation stream."""

    def __init__(self, args):
        super().__init__('repose_observe_viewer')
        self.args = args
        self.parts = OrderedDict()
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_labels = None
        self.latest_obs = None
        self.should_quit = False
        self.stage = 0
        self.attempt = 0
        self.count = 0
        self.pending_click = False
        self.pending_after_sequence = 0
        self.complete = False
        self.interlock = 'waiting for paired RGB-D'
        self.paired = 0
        self.started = time.monotonic()
        self.last_click_sequence = 0
        self.total_target = (
            len(COLOR_NAMES) * args.samples + args.negative_samples)

        best = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            CompressedImage, args.color_topic, self._on_color, best)
        self.create_subscription(Image, args.depth_topic, self._on_depth, best)
        self.create_subscription(Image, args.labels_topic, self._on_labels, best)
        self.create_subscription(
            CameraInfo, args.camera_info_topic, self._on_camera_info, best)
        self.create_subscription(
            CubeObservation, args.observation_topic, self._on_observation, best)
        self.create_subscription(UInt64, args.click_topic, self._on_click, best)

        snapshot = args.snapshot_prefix.rstrip('/')
        self.snapshot_color_pub = self.create_publisher(
            CompressedImage, snapshot + '/color/compressed', reliable)
        self.snapshot_depth_pub = self.create_publisher(
            Image, snapshot + '/depth', reliable)
        self.snapshot_labels_pub = self.create_publisher(
            Image, snapshot + '/labels', reliable)
        self.snapshot_camera_info_pub = self.create_publisher(
            CameraInfo, snapshot + '/camera_info', reliable)
        self.snapshot_observation_pub = self.create_publisher(
            CubeObservation, snapshot + '/observation', reliable)
        self.state_pub = self.create_publisher(
            CalibrationState, args.state_topic, reliable)
        self.create_timer(1.0 / 30.0, self._draw)

    def _slot(self, header):
        key = _stamp_ns(header)
        slot = self.parts.setdefault(key, {})
        self.parts.move_to_end(key)
        while len(self.parts) > 24:
            self.parts.popitem(last=False)
        return key, slot

    def _on_color(self, msg):
        data = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if image is None:
            return
        key, slot = self._slot(msg.header)
        slot['rgb'] = image
        slot['color_msg'] = msg
        self._try_pair(key)

    def _on_depth(self, msg):
        if msg.encoding not in ('16UC1', 'mono16'):
            self.get_logger().warning(
                f'unsupported depth encoding {msg.encoding}', once=True)
            return
        count = int(msg.height) * int(msg.width)
        raw = np.frombuffer(msg.data, dtype=np.uint16, count=count)
        if raw.size != count:
            return
        depth = raw.reshape(int(msg.height), int(msg.width)).copy()
        key, slot = self._slot(msg.header)
        slot['depth'] = depth
        slot['depth_msg'] = msg
        self._try_pair(key)

    def _on_labels(self, msg):
        if msg.encoding != 'mono8':
            return
        count = int(msg.height) * int(msg.width)
        raw = np.frombuffer(msg.data, dtype=np.uint8, count=count)
        if raw.size != count:
            return
        key, slot = self._slot(msg.header)
        slot['labels'] = raw.reshape(int(msg.height), int(msg.width)).copy()
        slot['labels_msg'] = msg
        self._try_pair(key)

    def _on_camera_info(self, msg):
        key, slot = self._slot(msg.header)
        slot['info'] = msg
        self._try_pair(key)

    def _on_observation(self, msg):
        key, slot = self._slot(msg.header)
        slot['obs'] = msg
        self._try_pair(key)

    def _try_pair(self, key):
        slot = self.parts.get(key)
        required = ('rgb', 'depth', 'labels', 'info', 'obs')
        if slot is None or slot.get('consumed') or any(k not in slot for k in required):
            return
        slot['consumed'] = True
        self.latest_rgb = slot['rgb']
        self.latest_depth = slot['depth']
        self.latest_labels = slot['labels']
        self.latest_obs = slot['obs']
        self.paired += 1
        self._consume(slot)

    def _target(self, stage=None):
        stage = self.stage if stage is None else stage
        if stage < len(COLOR_NAMES):
            return self.args.samples
        return self.args.negative_samples

    @staticmethod
    def _expected_color(stage):
        return stage if stage < len(COLOR_NAMES) else -1

    def _selected_total(self):
        completed = min(self.stage, len(COLOR_NAMES)) * self.args.samples
        if self.stage > len(COLOR_NAMES):
            completed += self.args.negative_samples
        return completed + (0 if self.complete else self.count)

    def _queue_click(self, source):
        if self.complete:
            self.interlock = f'{source} ignored: collection is complete'
            return
        if self.pending_click:
            self.interlock = f'{source} ignored: one click is already queued'
            return
        obs = self.latest_obs
        if obs is None or not obs.state_ready or not obs.stand_locked:
            self.interlock = f'{source} ignored: the controller is not in locked stand'
            return
        self.pending_click = True
        self.pending_after_sequence = int(obs.sequence)
        self.interlock = f'{source} received; waiting for next paired RGB-D frame'

    def _on_click(self, msg):
        sequence = int(msg.data)
        if sequence == self.last_click_sequence:
            return
        self.last_click_sequence = sequence
        self._queue_click(f'LT #{sequence}')

    def _publish_state(self, obs, stage, attempt, count, capturing, complete):
        state = CalibrationState()
        state.header = obs.header
        state.source_sequence = obs.sequence
        state.expected_color = (
            -1 if complete else int(self._expected_color(stage)))
        state.stage_index = int(stage)
        state.attempt = int(attempt)
        state.sample_index = int(count)
        target_stage = min(stage, len(STAGE_NAMES) - 1)
        state.samples_per_color = int(self._target(target_stage))
        state.capturing = bool(capturing)
        state.complete = bool(complete)
        self.state_pub.publish(state)

    def _publish_snapshot(self, slot):
        self.snapshot_color_pub.publish(slot['color_msg'])
        self.snapshot_depth_pub.publish(slot['depth_msg'])
        self.snapshot_camera_info_pub.publish(slot['info'])
        self.snapshot_labels_pub.publish(slot['labels_msg'])
        self.snapshot_observation_pub.publish(slot['obs'])

    def _consume(self, slot):
        obs = slot['obs']
        if self.complete:
            return

        if (self.pending_click and
                int(obs.sequence) <= self.pending_after_sequence):
            return

        ready = bool(obs.state_ready and obs.stand_locked)
        if not ready:
            if self.pending_click:
                self.pending_click = False
                self.interlock = 'CLICK CANCELLED: the controller left locked stand'
            return

        if not self.pending_click:
            if not self.interlock.startswith(('saved', 'stage complete')):
                self.interlock = 'ready: place cube, clear view, press LT'
            return

        self.pending_click = False
        self.count += 1
        self._publish_snapshot(slot)
        self._publish_state(obs, self.stage, self.attempt, self.count,
                            True, False)
        target = self._target()
        if self.count < target:
            self.interlock = (
                f'saved {STAGE_NAMES[self.stage].upper()} frame '
                f'{self.count}/{target}')
            return

        finished = self.stage
        self.stage += 1
        self.count = 0
        self.attempt = 0
        if self.stage >= len(STAGE_NAMES):
            self.complete = True
            self.interlock = (
                f'COMPLETE: {self.total_target} independent snapshots selected')
            self._publish_state(obs, self.stage, self.attempt, 0, False, True)
        else:
            self.interlock = (
                f'stage complete: {STAGE_NAMES[finished].upper()}; prepare '
                f'{STAGE_NAMES[self.stage].upper()}, then press LT')

    def _depth_panel(self, shape):
        if self.latest_depth is None:
            return np.zeros(shape, dtype=np.uint8)
        depth_m = self.latest_depth.astype(np.float32) * 1e-3
        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        scaled = np.zeros(depth_m.shape, dtype=np.uint8)
        if np.any(valid):
            clipped = np.clip(depth_m, self.args.depth_min, self.args.depth_max)
            scaled[valid] = np.asarray(
                255.0 * (clipped[valid] - self.args.depth_min) /
                max(self.args.depth_max - self.args.depth_min, 1e-6),
                dtype=np.uint8)
        panel = cv2.applyColorMap(255 - scaled, cv2.COLORMAP_TURBO)
        panel[~valid] = 0
        return cv2.resize(panel, (shape[1], shape[0]), interpolation=cv2.INTER_NEAREST)

    def _rgb_panel(self, shape):
        if self.latest_rgb is None:
            return np.zeros(shape, dtype=np.uint8)
        rgb = cv2.resize(self.latest_rgb, (shape[1], shape[0]))
        if self.latest_labels is None:
            return rgb
        labels = cv2.resize(self.latest_labels, (shape[1], shape[0]),
                            interpolation=cv2.INTER_NEAREST)
        mask = labels < len(LABEL_BGR)
        paint = np.zeros_like(rgb)
        paint[mask] = LABEL_BGR[labels[mask]]
        output = rgb.copy()
        output[mask] = cv2.addWeighted(rgb, 0.55, paint, 0.45, 0.0)[mask]
        return output

    def _draw(self):
        height = self.args.height
        width = self.args.width
        shape = (height, width, 3)
        rgb = self._rgb_panel(shape)
        depth = self._depth_panel(shape)
        obs = self.latest_obs

        rate = self.paired / max(time.monotonic() - self.started, 1e-6)
        rgb = _lines(
            rgb,
            ['RGB + classified pixels',
             f'stream frames {self.paired} | paired {rate:.1f} Hz'])
        depth = _lines(
            depth,
            ['depth (metres)',
             f'{self.args.depth_min:.2f} m .. {self.args.depth_max:.2f} m'])

        status = np.zeros(shape, dtype=np.uint8)
        if self.complete:
            expected = 'DONE'
            progress = (
                f'selected frames: {self._selected_total()} / '
                f'{self.total_target}')
        else:
            expected = STAGE_NAMES[self.stage].upper()
            progress = (
                f'selected frames: {self.count} / {self._target()} this stage '
                f'| {self._selected_total()} / {self.total_target} total '
                f'| attempt {self.attempt}')
        observed = '--'
        result = 'waiting for observation'
        geometry = ''
        lock = 'the controller: unknown'
        if obs is not None:
            if 0 <= obs.color < len(COLOR_NAMES):
                observed = COLOR_NAMES[obs.color].upper()
            result = (f'observed {observed} | color={int(obs.color_ok)} '
                      f'pose={int(obs.pose_ok)} | {obs.reason}')
            geometry = (f'r={obs.range_m:.2f}m b={np.degrees(obs.bearing_rad):+.1f}deg '
                        f'phi={np.degrees(obs.phi_rad):+.1f}deg n={obs.n_px}')
            lock = ('the controller: nominal lock OK' if obs.stand_locked else
                    'the controller: NOT IN LOCKED STAND')
        mode = 'CLICK QUEUED' if self.pending_click else 'READY FOR LT'
        if self.complete:
            mode = 'COMPLETE'
        status_color = (0, 255, 0) if self.pending_click else (0, 200, 255)
        status = _lines(status, [
            f'EXPECTED SAMPLE: {expected}', progress, mode, lock,
            result, geometry, self.interlock,
            'LT / Space click | p cancel | r retry stage | q quit',
        ], color=status_color, scale=0.55)

        cv2.imshow('The planner observation calibration', np.hstack((rgb, depth, status)))
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            self.should_quit = True
        elif key == ord('p'):
            self.pending_click = False
            self.pending_after_sequence = 0
            self.interlock = 'queued click cancelled'
        elif key == ord('r') and not self.complete:
            self.pending_click = False
            self.pending_after_sequence = 0
            self.count = 0
            self.attempt += 1
            self.interlock = (
                f'{STAGE_NAMES[self.stage].upper()} retry; press LT when ready')
        elif key == ord(' ') and not self.complete:
            self._queue_click('Space')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--samples', type=int, default=30,
                        help='positive snapshots per color (default: 30)')
    parser.add_argument('--negative-samples', type=int, default=60,
                        help='negative snapshots after pink (default: 60)')
    parser.add_argument('--width', type=int, default=480)
    parser.add_argument('--height', type=int, default=360)
    parser.add_argument('--depth-min', type=float, default=0.15)
    parser.add_argument('--depth-max', type=float, default=2.0)
    parser.add_argument('--color-topic',
                        default='/vibe/planner/observe/color/compressed')
    parser.add_argument('--depth-topic', default='/vibe/planner/observe/depth')
    parser.add_argument('--labels-topic', default='/vibe/planner/observe/labels')
    parser.add_argument('--camera-info-topic',
                        default='/vibe/planner/observe/camera_info')
    parser.add_argument('--observation-topic',
                        default='/vibe/planner/observe/observation')
    parser.add_argument('--click-topic',
                        default='/vibe/planner/calibration/click')
    parser.add_argument('--snapshot-prefix',
                        default='/vibe/planner/calibration')
    parser.add_argument('--state-topic', default='/vibe/planner/calibration/state')
    args, ros_args = parser.parse_known_args()
    if args.samples <= 0:
        parser.error('--samples must be positive')
    if args.negative_samples <= 0:
        parser.error('--negative-samples must be positive')
    if not args.snapshot_prefix.startswith('/'):
        parser.error('--snapshot-prefix must be an absolute ROS topic prefix')
    if args.depth_min < 0.0 or args.depth_max <= args.depth_min:
        parser.error('need 0 <= --depth-min < --depth-max')

    rclpy.init(args=ros_args)
    node = ObserveViewer(args)
    try:
        while rclpy.ok() and not node.should_quit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
