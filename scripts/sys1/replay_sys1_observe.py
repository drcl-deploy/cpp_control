#!/usr/bin/env python3
"""Read-only, frame-steppable replay viewer for Sys1 calibration bags."""

import argparse
from collections import defaultdict
import os
from pathlib import Path
import sys

import cv2
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import yaml

# Importing the pip OpenCV wheel can point Qt at OpenCV's private xcb plugin.
# This application uses PyQt's plugins; OpenCV is only used for image arrays.
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = QtCore.QLibraryInfo.location(
    QtCore.QLibraryInfo.PluginsPath,
)
if '/cv2/qt/' in os.environ.get('QT_QPA_FONTDIR', ''):
    os.environ.pop('QT_QPA_FONTDIR')


COLOR_NAMES = ('red', 'orange', 'green', 'yellow', 'blue', 'pink')
STAGE_NAMES = COLOR_NAMES + ('negative / no cube',)
LABEL_BGR = np.asarray([
    (51, 51, 230), (26, 115, 242), (51, 230, 51),
    (51, 230, 230), (230, 51, 51), (166, 51, 204),
], dtype=np.uint8)

STATE_TOPIC = '/vibe/sys1/calibration/state'
COLOR_TOPIC = '/vibe/sys1/calibration/color/compressed'
DEPTH_TOPIC = '/vibe/sys1/calibration/depth'
LABELS_TOPIC = '/vibe/sys1/calibration/labels'
CAMERA_INFO_TOPIC = '/vibe/sys1/calibration/camera_info'
OBSERVATION_TOPIC = '/vibe/sys1/calibration/observation'
REQUIRED_TOPICS = (
    STATE_TOPIC,
    COLOR_TOPIC,
    DEPTH_TOPIC,
    LABELS_TOPIC,
    CAMERA_INFO_TOPIC,
    OBSERVATION_TOPIC,
)


def _stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def _metadata(path):
    metadata_path = path / 'metadata.yaml'
    try:
        with metadata_path.open('r', encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
        return document['rosbag2_bagfile_information']
    except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
        raise RuntimeError(
            f'cannot read rosbag metadata {metadata_path}: {error}') from error


def _normalize_bag_path(value):
    path = Path(value).expanduser().resolve()
    if path.is_file():
        path = path.parent
    if not (path / 'metadata.yaml').is_file():
        raise RuntimeError(f'not a completed ROS bag directory: {path}')
    return path


def _latest_bag(root):
    root = Path(root).expanduser().resolve()
    if not root.is_dir():
        raise RuntimeError(f'bag root does not exist: {root}')
    candidates = [
        path for path in root.iterdir()
        if path.is_dir() and (path / 'metadata.yaml').is_file()
    ]
    if not candidates:
        raise RuntimeError(f'no completed Sys1 observation bags under {root}')
    return max(candidates, key=lambda path: (path / 'metadata.yaml').stat().st_mtime)


def _storage_options(uri, storage_id):
    return rosbag2_py.StorageOptions(uri=str(uri), storage_id=storage_id)


def _converter_options():
    return rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )


def _color_name(value):
    value = int(value)
    return COLOR_NAMES[value] if 0 <= value < len(COLOR_NAMES) else '--'


class CalibrationBag:
    """Load the final selected attempt and join every exact RGB-D snapshot."""

    def __init__(self, path):
        self.path = _normalize_bag_path(path)
        info = _metadata(self.path)
        self.storage_id = str(info.get('storage_identifier', 'sqlite3'))
        self.samples = self._load()

    def _reader(self):
        reader = rosbag2_py.SequentialReader()
        try:
            reader.open(
                _storage_options(self.path, self.storage_id),
                _converter_options(),
            )
        except Exception as error:
            raise RuntimeError(
                f"cannot open {self.storage_id} bag '{self.path}': {error}"
            ) from error
        return reader

    def _load(self):
        reader = self._reader()
        topic_types = {
            item.name: item.type for item in reader.get_all_topics_and_types()
        }
        missing = sorted(set(REQUIRED_TOPICS) - set(topic_types))
        if missing:
            raise RuntimeError(
                'not a clicked Sys1 observation bag; missing topics: ' +
                ', '.join(missing))
        classes = {
            topic: get_message(topic_types[topic]) for topic in REQUIRED_TOPICS
        }
        states = []
        by_stamp = {topic: {} for topic in REQUIRED_TOPICS[1:]}
        while reader.has_next():
            topic, serialized, bag_timestamp = reader.read_next()
            if topic not in classes:
                continue
            message = deserialize_message(serialized, classes[topic])
            if topic == STATE_TOPIC:
                if message.capturing:
                    states.append((int(bag_timestamp), message))
            else:
                by_stamp[topic][_stamp_ns(message.header)] = message

        if not states:
            raise RuntimeError('bag has no selected calibration snapshots')

        # A stage retry intentionally leaves the abandoned attempt in the bag.
        # Match the validator: display only the final attempt for each label.
        final_attempt = defaultdict(lambda: -1)
        for _, state in states:
            truth = int(state.expected_color)
            final_attempt[truth] = max(final_attempt[truth], int(state.attempt))
        states = [
            item for item in states
            if int(item[1].attempt) == final_attempt[int(item[1].expected_color)]
        ]
        states.sort(key=lambda item: (
            int(item[1].stage_index), int(item[1].sample_index), item[0]))

        samples = []
        errors = []
        for _, state in states:
            stamp = _stamp_ns(state.header)
            parts = {topic: messages.get(stamp)
                     for topic, messages in by_stamp.items()}
            absent = [topic for topic, message in parts.items()
                      if message is None]
            if absent:
                errors.append(
                    f'stage {state.stage_index} sample {state.sample_index}: '
                    f'missing {", ".join(absent)}')
                continue
            observation = parts[OBSERVATION_TOPIC]
            if int(observation.sequence) != int(state.source_sequence):
                errors.append(
                    f'stage {state.stage_index} sample {state.sample_index}: '
                    'observation sequence mismatch')
                continue
            samples.append({
                'state': state,
                'color': parts[COLOR_TOPIC],
                'depth': parts[DEPTH_TOPIC],
                'labels': parts[LABELS_TOPIC],
                'camera_info': parts[CAMERA_INFO_TOPIC],
                'observation': observation,
            })
        if errors:
            detail = '\n  '.join(errors[:10])
            remainder = '' if len(errors) <= 10 else (
                f'\n  ... {len(errors) - 10} more')
            raise RuntimeError(
                f'bag has {len(errors)} incomplete snapshot(s):\n  '
                f'{detail}{remainder}')
        return samples


def _decode_color(message):
    image = cv2.imdecode(
        np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError('cannot decode selected RGB frame')
    return image


def _decode_depth(message):
    if message.encoding not in ('16UC1', 'mono16'):
        raise RuntimeError(f'unsupported depth encoding: {message.encoding}')
    dtype = np.dtype('>u2' if message.is_bigendian else '<u2')
    row_values = int(message.step) // dtype.itemsize
    raw = np.frombuffer(message.data, dtype=dtype)
    needed = int(message.height) * row_values
    if raw.size < needed or row_values < int(message.width):
        raise RuntimeError('truncated depth image')
    return raw[:needed].reshape(int(message.height), row_values)[
        :, :int(message.width)].astype(np.uint16, copy=False)


def _decode_labels(message):
    if message.encoding != 'mono8':
        raise RuntimeError(f'unsupported labels encoding: {message.encoding}')
    row_values = int(message.step)
    raw = np.frombuffer(message.data, dtype=np.uint8)
    needed = int(message.height) * row_values
    if raw.size < needed or row_values < int(message.width):
        raise RuntimeError('truncated labels image')
    return raw[:needed].reshape(int(message.height), row_values)[
        :, :int(message.width)]


def _pixmap(bgr):
    rgb = np.ascontiguousarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    height, width = rgb.shape[:2]
    image = QtGui.QImage(
        rgb.data, width, height, width * 3, QtGui.QImage.Format_RGB888)
    return QtGui.QPixmap.fromImage(image.copy())


class ImagePanel(QtWidgets.QGroupBox):
    """Display one fixed-size, aspect-preserving OpenCV image."""

    def __init__(self, title):
        super().__init__(title)
        layout = QtWidgets.QVBoxLayout(self)
        self.image = QtWidgets.QLabel()
        self.image.setAlignment(QtCore.Qt.AlignCenter)
        self.image.setFixedSize(480, 360)
        self.image.setStyleSheet('background: #111;')
        layout.addWidget(self.image)

    def show_bgr(self, image, smooth=True):
        mode = (QtCore.Qt.SmoothTransformation if smooth else
                QtCore.Qt.FastTransformation)
        pixmap = _pixmap(image).scaled(
            self.image.size(), QtCore.Qt.KeepAspectRatio, mode)
        self.image.setPixmap(pixmap)


class ReplayWindow(QtWidgets.QMainWindow):
    """Navigate the exact labeled snapshots in one calibration bag."""

    def __init__(self, bag, depth_min, depth_max, fps):
        super().__init__()
        self.bag = bag
        self.depth_min = depth_min
        self.depth_max = depth_max
        self.fps = fps
        self.shortcuts = []
        self.setWindowTitle(f'Sys1 observation replay — {bag.path.name}')

        center = QtWidgets.QWidget()
        self.setCentralWidget(center)
        outer = QtWidgets.QVBoxLayout(center)

        self.summary = QtWidgets.QLabel()
        self.summary.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        outer.addWidget(self.summary)

        panels = QtWidgets.QHBoxLayout()
        self.rgb_panel = ImagePanel('RGB + classified pixels')
        self.depth_panel = ImagePanel(
            f'Depth ({depth_min:.2f} m .. {depth_max:.2f} m)')
        panels.addWidget(self.rgb_panel)
        panels.addWidget(self.depth_panel)

        status_box = QtWidgets.QGroupBox('Recorded label and production result')
        status_layout = QtWidgets.QVBoxLayout(status_box)
        self.result = QtWidgets.QLabel()
        self.result.setWordWrap(True)
        self.result.setMinimumHeight(58)
        status_layout.addWidget(self.result)
        self.status = QtWidgets.QPlainTextEdit()
        self.status.setReadOnly(True)
        self.status.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.status.setFixedSize(440, 292)
        font = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont)
        self.status.setFont(font)
        status_layout.addWidget(self.status)
        panels.addWidget(status_box)
        outer.addLayout(panels)

        controls = QtWidgets.QHBoxLayout()
        self.previous_stage_button = QtWidgets.QPushButton('Previous stage')
        self.previous_button = QtWidgets.QPushButton('Previous frame')
        self.play_button = QtWidgets.QPushButton('Play')
        self.play_button.setCheckable(True)
        self.next_button = QtWidgets.QPushButton('Next frame')
        self.next_stage_button = QtWidgets.QPushButton('Next stage')
        self.stage_combo = QtWidgets.QComboBox()
        self.overlay = QtWidgets.QCheckBox('Label overlay')
        self.overlay.setChecked(True)
        for widget in (
                self.previous_stage_button, self.previous_button,
                self.play_button, self.next_button, self.next_stage_button):
            controls.addWidget(widget)
        controls.addWidget(QtWidgets.QLabel('Jump to:'))
        controls.addWidget(self.stage_combo)
        controls.addStretch(1)
        controls.addWidget(self.overlay)
        outer.addLayout(controls)

        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, len(bag.samples) - 1)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(10)
        outer.addWidget(self.slider)

        self.stage_starts = []
        previous_stage = None
        for index, sample in enumerate(bag.samples):
            stage = int(sample['state'].stage_index)
            if stage != previous_stage:
                self.stage_starts.append(index)
                name = (STAGE_NAMES[stage] if 0 <= stage < len(STAGE_NAMES)
                        else f'stage {stage}')
                self.stage_combo.addItem(name, index)
                previous_stage = stage

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(max(1, int(round(1000.0 / fps))))
        self.timer.timeout.connect(self._advance)
        self.slider.valueChanged.connect(self._render)
        self.previous_button.clicked.connect(self._previous)
        self.next_button.clicked.connect(self._next)
        self.previous_stage_button.clicked.connect(self._previous_stage)
        self.next_stage_button.clicked.connect(self._next_stage)
        self.play_button.toggled.connect(self._toggle_play)
        self.stage_combo.activated.connect(self._jump_combo)
        self.overlay.toggled.connect(lambda _: self._render(self.slider.value()))

        self._shortcut(QtCore.Qt.Key_Left, self._previous)
        self._shortcut(QtCore.Qt.Key_Right, self._next)
        self._shortcut(QtCore.Qt.Key_PageUp, self._previous_stage)
        self._shortcut(QtCore.Qt.Key_PageDown, self._next_stage)
        self._shortcut(QtCore.Qt.Key_Home, lambda: self.slider.setValue(0))
        self._shortcut(
            QtCore.Qt.Key_End,
            lambda: self.slider.setValue(len(self.bag.samples) - 1))
        self._shortcut(QtCore.Qt.Key_Space, self.play_button.toggle)

        self._render(0)

    def _shortcut(self, key, callback):
        shortcut = QtWidgets.QShortcut(QtGui.QKeySequence(key), self)
        shortcut.activated.connect(callback)
        self.shortcuts.append(shortcut)

    def _toggle_play(self, playing):
        self.play_button.setText('Pause' if playing else 'Play')
        if playing:
            self.timer.start()
        else:
            self.timer.stop()

    def _advance(self):
        if self.slider.value() >= self.slider.maximum():
            self.play_button.setChecked(False)
            return
        self.slider.setValue(self.slider.value() + 1)

    def _previous(self):
        self.slider.setValue(max(0, self.slider.value() - 1))

    def _next(self):
        self.slider.setValue(min(
            self.slider.maximum(), self.slider.value() + 1))

    def _previous_stage(self):
        current = self.slider.value()
        targets = [index for index in self.stage_starts if index < current]
        self.slider.setValue(targets[-1] if targets else 0)

    def _next_stage(self):
        current = self.slider.value()
        targets = [index for index in self.stage_starts if index > current]
        if targets:
            self.slider.setValue(targets[0])

    def _jump_combo(self, combo_index):
        self.slider.setValue(int(self.stage_combo.itemData(combo_index)))

    def _render(self, index):
        sample = self.bag.samples[index]
        state = sample['state']
        observation = sample['observation']

        rgb = _decode_color(sample['color'])
        labels = _decode_labels(sample['labels'])
        if self.overlay.isChecked():
            labels_large = cv2.resize(
                labels, (rgb.shape[1], rgb.shape[0]),
                interpolation=cv2.INTER_NEAREST)
            mask = labels_large < len(LABEL_BGR)
            paint = np.zeros_like(rgb)
            paint[mask] = LABEL_BGR[labels_large[mask]]
            rgb[mask] = cv2.addWeighted(
                rgb, 0.55, paint, 0.45, 0.0)[mask]
        self.rgb_panel.show_bgr(rgb)

        depth = _decode_depth(sample['depth'])
        depth_m = depth.astype(np.float32) * 1e-3
        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        scaled = np.zeros(depth.shape, dtype=np.uint8)
        if np.any(valid):
            clipped = np.clip(depth_m, self.depth_min, self.depth_max)
            scaled[valid] = np.asarray(
                255.0 * (clipped[valid] - self.depth_min) /
                (self.depth_max - self.depth_min), dtype=np.uint8)
        depth_bgr = cv2.applyColorMap(255 - scaled, cv2.COLORMAP_TURBO)
        depth_bgr[~valid] = 0
        self.depth_panel.show_bgr(depth_bgr, smooth=False)

        truth = int(state.expected_color)
        observed = int(observation.color)
        truth_name = (_color_name(truth) if truth >= 0 else
                      'negative / no cube')
        observed_name = _color_name(observed)
        correct = ((truth < 0 and not observation.color_ok) or
                   (truth >= 0 and observation.color_ok and observed == truth))
        verdict = 'MATCH' if correct else 'MISMATCH'
        verdict_color = '#15803d' if correct else '#b91c1c'
        self.result.setText(
            f'<b style="color:{verdict_color}">{verdict}</b> &nbsp; '
            f'expected <b>{truth_name.upper()}</b> &nbsp; '
            f'observed <b>{observed_name.upper()}</b>')

        position = tuple(float(value) for value in observation.position)
        lines = [
            f'frame         {index + 1}/{len(self.bag.samples)}',
            f'stage/sample  {state.stage_index} / '
            f'{state.sample_index}/{state.samples_per_color}',
            f'attempt       {state.attempt}',
            f'sequence      {state.source_sequence}',
            '',
            f'color_ok      {int(observation.color_ok)}',
            f'pose_ok       {int(observation.pose_ok)}',
            f'reason        {observation.reason}',
            f'pixels        {observation.n_px}',
            f'range         {observation.range_m:.3f} m',
            f'bearing       {np.degrees(observation.bearing_rad):+.2f} deg',
            f'phi           {np.degrees(observation.phi_rad):+.2f} deg',
            f'position      ({position[0]:+.3f}, {position[1]:+.3f}, '
            f'{position[2]:+.3f}) m',
            f'depth valid   {100.0 * np.mean(valid):.1f}%',
            '',
            'candidates     color  pixels  up_dot  visible  big',
        ]
        count = min(
            len(observation.candidate_color),
            len(observation.candidate_pixels),
            len(observation.candidate_up_dot),
            len(observation.candidate_visible),
            len(observation.candidate_big),
        )
        if count == 0:
            lines.append('               --')
        for candidate in range(count):
            lines.append(
                f'               {_color_name(observation.candidate_color[candidate]):>6s}'
                f'  {observation.candidate_pixels[candidate]:6d}'
                f'  {observation.candidate_up_dot[candidate]:6.3f}'
                f'  {observation.candidate_visible[candidate]:7.3f}'
                f'  {observation.candidate_big[candidate]:5.3f}')
        self.status.setPlainText('\n'.join(lines))

        stage = int(state.stage_index)
        stage_start = max(start for start in self.stage_starts if start <= index)
        combo_index = self.stage_combo.findData(stage_start)
        self.stage_combo.blockSignals(True)
        self.stage_combo.setCurrentIndex(combo_index)
        self.stage_combo.blockSignals(False)
        self.summary.setText(
            f'{self.bag.path}  |  {len(self.bag.samples)} selected frames  |  '
            f'{STAGE_NAMES[stage] if 0 <= stage < len(STAGE_NAMES) else stage}  '
            '|  Left/Right: frame  PageUp/PageDown: stage  Space: play/pause')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'bag', nargs='?',
        help='completed bag directory or its .db3/.mcap file; default: newest')
    parser.add_argument('--depth-min', type=float, default=0.15)
    parser.add_argument('--depth-max', type=float, default=2.0)
    parser.add_argument('--fps', type=float, default=5.0,
                        help='playback frames per second (default: 5)')
    args = parser.parse_args()
    if args.depth_min < 0.0 or args.depth_max <= args.depth_min:
        parser.error('need 0 <= --depth-min < --depth-max')
    if args.fps <= 0.0:
        parser.error('--fps must be positive')

    try:
        bag_path = (args.bag if args.bag else _latest_bag(
            os.environ.get('SYS1_OBSERVE_BAG_DIR', './sys1_observe_bags')))
        bag = CalibrationBag(bag_path)
    except RuntimeError as error:
        print(f'replay_sys1_observe: {error}', file=sys.stderr)
        return 1

    application = QtWidgets.QApplication(sys.argv[:1])
    window = ReplayWindow(bag, args.depth_min, args.depth_max, args.fps)
    window.show()
    return application.exec_()


if __name__ == '__main__':
    sys.exit(main())
