#!/usr/bin/env python3
"""Read-only, synchronized replay viewer for Vibe experiment bags."""

import argparse
from array import array
import os
from pathlib import Path
import sys

import cv2
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from rclpy.logging import LoggingSeverity, set_logger_level
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import yaml

# The pip OpenCV wheel rewrites Qt's platform-plugin path at import time. That
# points a PyQt application at OpenCV's private xcb plugin and aborts before a
# window is created. Use the plugins belonging to the PyQt runtime instead;
# OpenCV is only used here for array operations, never for its own GUI.
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = QtCore.QLibraryInfo.location(
    QtCore.QLibraryInfo.PluginsPath,
)
if '/cv2/qt/' in os.environ.get('QT_QPA_FONTDIR', ''):
    os.environ.pop('QT_QPA_FONTDIR')


FRAME_TOPIC = '/enc/frame'
TOKENS_TOPIC = '/enc/tokens'
ATTENTION_TOPIC = '/vibe/sonic/attention_mask'
LOWSTATE_TOPIC = '/lowstate'
LOWCMD_TOPIC = '/lowcmd'
VIEW_TOPICS = (
    FRAME_TOPIC,
    TOKENS_TOPIC,
    ATTENTION_TOPIC,
    LOWSTATE_TOPIC,
    LOWCMD_TOPIC,
)
NANOSECONDS_PER_SECOND = 1_000_000_000

# unitree_hg motor order for the actuated G1 joints. Keep this synchronized
# with include/common/g1/joint_orders.hpp and config/vibe/g1_vibe_sonic.yaml.
G1_JOINTS = (
    'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
    'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
    'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_roll_joint',
    'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint', 'right_elbow_joint',
    'right_wrist_roll_joint', 'right_wrist_pitch_joint',
    'right_wrist_yaw_joint',
)


def _metadata(path):
    metadata_path = path / 'metadata.yaml'
    try:
        with metadata_path.open('r', encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
        return document['rosbag2_bagfile_information']
    except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
        raise RuntimeError(f'cannot read rosbag metadata {metadata_path}: {error}') from error


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
        raise RuntimeError(f'no completed ROS bags under {root}')
    return max(candidates, key=lambda path: (path / 'metadata.yaml').stat().st_mtime)


def _storage_options(uri, storage_id):
    return rosbag2_py.StorageOptions(uri=str(uri), storage_id=storage_id)


def _converter_options():
    return rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )


def _storage_filter(topics):
    return rosbag2_py.StorageFilter(topics=list(topics))


class BagAccess:
    """Bag metadata and independent filtered readers for safe random seeking."""

    def __init__(self, path):
        self.path = _normalize_bag_path(path)
        info = _metadata(self.path)
        self.storage_id = str(info.get('storage_identifier', 'sqlite3'))
        self.start_ns = int(info['starting_time']['nanoseconds_since_epoch'])
        self.duration_ns = int(info['duration']['nanoseconds'])
        self.end_ns = self.start_ns + self.duration_ns

        reader = self.open_reader()
        self.topic_types = {
            item.name: item.type for item in reader.get_all_topics_and_types()
        }
        self.present_topics = set(self.topic_types)

    def open_reader(self, topics=None):
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
        if topics:
            reader.set_filter(_storage_filter(topics))
        return reader

    def message_class(self, topic):
        type_name = self.topic_types.get(topic)
        if not type_name:
            return None
        try:
            return get_message(type_name)
        except (AttributeError, ImportError, ModuleNotFoundError, ValueError) as error:
            raise RuntimeError(
                f'message support for {topic} ({type_name}) is unavailable: {error}'
            ) from error


class TopicSeeker:
    """Find a topic sample nearest a requested bag timestamp."""

    def __init__(self, bag, topic):
        self.bag = bag
        self.topic = topic
        self.message_class = bag.message_class(topic)
        self.reader = bag.open_reader([topic]) if self.message_class else None
        self.start_ns = bag.start_ns

    @property
    def available(self):
        return self.reader is not None

    def nearest(self, target_ns):
        if not self.available:
            return None, None

        # Vibe streams are capped at 50 Hz. Seeking slightly before the cursor
        # lets us compare the neighboring samples instead of always choosing
        # the first sample after it.
        seek_ns = max(self.start_ns, target_ns - 100_000_000)
        can_seek = hasattr(self.reader, 'seek')
        if can_seek:
            self.reader.seek(seek_ns)
        else:
            # Foxy's Python reader has no seek binding. Reopening is slower,
            # but preserves a functional replay path without republishing.
            self.reader = self.bag.open_reader([self.topic])
        best = None
        best_delta = None
        reads = 0
        while self.reader.has_next() and (not can_seek or reads < 64):
            topic, serialized, timestamp_ns = self.reader.read_next()
            reads += 1
            if topic != self.topic:
                continue
            if timestamp_ns < seek_ns:
                continue
            delta = abs(timestamp_ns - target_ns)
            if best_delta is None or delta < best_delta:
                best = (serialized, timestamp_ns)
                best_delta = delta
            if timestamp_ns >= target_ns:
                break

        if best is None:
            return None, None
        return deserialize_message(best[0], self.message_class), best[1]


class FieldSpec:
    """A user-facing scalar field and its ROS message accessor."""

    def __init__(self, label, getter):
        self.label = label
        self.getter = getter


def _lowstate_fields():
    fields = [
        FieldSpec('tick', lambda msg: msg.tick),
        FieldSpec('mode_pr', lambda msg: msg.mode_pr),
        FieldSpec('mode_machine', lambda msg: msg.mode_machine),
    ]
    for index, name in enumerate(('w', 'x', 'y', 'z')):
        fields.append(FieldSpec(
            f'imu.quaternion.{name}',
            lambda msg, i=index: msg.imu_state.quaternion[i],
        ))
    for member, names in (
            ('rpy', ('roll', 'pitch', 'yaw')),
            ('gyroscope', ('x', 'y', 'z')),
            ('accelerometer', ('x', 'y', 'z'))):
        for index, name in enumerate(names):
            fields.append(FieldSpec(
                f'imu.{member}.{name}',
                lambda msg, key=member, i=index: getattr(msg.imu_state, key)[i],
            ))
    fields.append(FieldSpec(
        'imu.temperature', lambda msg: msg.imu_state.temperature,
    ))
    for index, joint in enumerate(G1_JOINTS):
        for member in ('q', 'dq', 'ddq', 'tau_est', 'vol'):
            fields.append(FieldSpec(
                f'{joint}.{member}',
                lambda msg, i=index, key=member: getattr(msg.motor_state[i], key),
            ))
    return fields


def _lowcmd_fields():
    fields = [
        FieldSpec('mode_pr', lambda msg: msg.mode_pr),
        FieldSpec('mode_machine', lambda msg: msg.mode_machine),
    ]
    for index, joint in enumerate(G1_JOINTS):
        for member in ('q', 'dq', 'tau', 'kp', 'kd'):
            fields.append(FieldSpec(
                f'{joint}.{member}',
                lambda msg, i=index, key=member: getattr(msg.motor_cmd[i], key),
            ))
    return fields


class Telemetry:
    """Compact, plot-ready scalar telemetry loaded without retaining messages."""

    def __init__(self, bag):
        self.start_ns = bag.start_ns
        self.times = {}
        self.values = {}
        field_sets = {
            LOWSTATE_TOPIC: _lowstate_fields(),
            LOWCMD_TOPIC: _lowcmd_fields(),
        }
        topics = [topic for topic in field_sets if topic in bag.present_topics]
        if not topics:
            return

        message_classes = {topic: bag.message_class(topic) for topic in topics}
        time_buffers = {topic: array('q') for topic in topics}
        value_buffers = {
            topic: {field.label: array('f') for field in field_sets[topic]}
            for topic in topics
        }
        reader = bag.open_reader(topics)
        while reader.has_next():
            topic, serialized, timestamp_ns = reader.read_next()
            if topic not in message_classes:
                continue
            message = deserialize_message(serialized, message_classes[topic])
            time_buffers[topic].append(timestamp_ns)
            for field in field_sets[topic]:
                value_buffers[topic][field.label].append(float(field.getter(message)))

        for topic in topics:
            timestamps = np.frombuffer(time_buffers[topic], dtype=np.int64)
            self.times[topic] = (timestamps - self.start_ns) / NANOSECONDS_PER_SECOND
            self.values[topic] = {
                name: np.frombuffer(values, dtype=np.float32)
                for name, values in value_buffers[topic].items()
            }

    def fields(self, topic):
        return tuple(self.values.get(topic, {}))


def _image_to_bgr(message):
    if message is None or message.encoding not in ('bgr8', 'rgb8'):
        return None
    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    data = np.frombuffer(message.data, dtype=np.uint8)
    if height <= 0 or width <= 0 or step < width * 3 or data.size < height * step:
        return None
    image = data[:height * step].reshape(height, step)[:, :width * 3]
    image = image.reshape(height, width, 3).copy()
    if message.encoding == 'rgb8':
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image


def _time_delta(sample_ns, target_ns):
    if sample_ns is None:
        return 'missing'
    return f'Δ {((sample_ns - target_ns) / 1e6):+.1f} ms'


def _pixmap(image):
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    height, width, _ = rgb.shape
    qimage = QtGui.QImage(
        rgb.data, width, height, width * 3, QtGui.QImage.Format_RGB888,
    ).copy()
    return QtGui.QPixmap.fromImage(qimage)


class ImagePanel(QtWidgets.QGroupBox):
    """Aspect-preserving image panel with a sample status line."""

    def __init__(self, title):
        super().__init__(title)
        self.image = None
        self.label = QtWidgets.QLabel('waiting for sample')
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setMinimumSize(320, 220)
        self.status = QtWidgets.QLabel('')
        self.status.setAlignment(QtCore.Qt.AlignCenter)
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.label, 1)
        layout.addWidget(self.status)

    def set_image(self, image, status):
        self.image = image
        self.status.setText(status)
        if image is None:
            self.label.setPixmap(QtGui.QPixmap())
            self.label.setText('topic or supported image sample unavailable')
        else:
            self.label.setText('')
            self._scale_image()

    def _scale_image(self):
        if self.image is None:
            return
        pixmap = _pixmap(self.image).scaled(
            self.label.size(), QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation,
        )
        self.label.setPixmap(pixmap)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._scale_image()


class AttentionPanel(ImagePanel):
    """Image panel with mean or per-query attention selection."""

    def __init__(self):
        super().__init__(ATTENTION_TOPIC)
        self.selector = QtWidgets.QComboBox()
        self.selector.addItem('mean(all queries)')
        self.layout().insertWidget(0, self.selector)

    def set_queries(self, names):
        current = self.selector.currentText()
        choices = ['mean(all queries)'] + list(names)
        existing = [self.selector.itemText(i) for i in range(self.selector.count())]
        if choices == existing:
            return
        self.selector.blockSignals(True)
        self.selector.clear()
        self.selector.addItems(choices)
        index = self.selector.findText(current)
        self.selector.setCurrentIndex(max(0, index))
        self.selector.blockSignals(False)


class TelemetryPanel(QtWidgets.QGroupBox):
    """Selectable full-duration scalar trace with a shared time cursor."""

    def __init__(self, topic, telemetry, default_field):
        super().__init__(topic)
        self.topic = topic
        self.telemetry = telemetry
        self.selector = QtWidgets.QComboBox()
        self.selector.setEditable(True)
        self.selector.setInsertPolicy(QtWidgets.QComboBox.NoInsert)
        self.selector.addItems(telemetry.fields(topic))
        completer = self.selector.completer()
        if completer:
            completer.setCaseSensitivity(QtCore.Qt.CaseInsensitive)
            completer.setFilterMode(QtCore.Qt.MatchContains)

        index = self.selector.findText(default_field)
        if index >= 0:
            self.selector.setCurrentIndex(index)

        self.figure = Figure(figsize=(5, 3), tight_layout=True)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.axes = self.figure.add_subplot(111)
        self.value_label = QtWidgets.QLabel('')
        self.value_label.setAlignment(QtCore.Qt.AlignCenter)
        self.cursor = None
        self.marker = None
        self.plot_background = None
        self.current_seconds = 0.0
        self.selector.currentIndexChanged.connect(self._plot_field)
        self.canvas.mpl_connect('draw_event', self._on_canvas_draw)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.selector)
        layout.addWidget(self.value_label)
        layout.addWidget(self.canvas, 1)
        self._plot_field()

    def _plot_field(self):
        self.plot_background = None
        self.axes.clear()
        fields = self.telemetry.values.get(self.topic, {})
        times = self.telemetry.times.get(self.topic, np.empty(0))
        field = self.selector.currentText()
        values = fields.get(field)
        if values is None or times.size == 0:
            self.axes.text(
                0.5, 0.5, 'topic unavailable', ha='center', va='center',
                transform=self.axes.transAxes,
            )
            self.cursor = None
            self.marker = None
            self.value_label.setText('topic unavailable')
        else:
            self.axes.plot(times, values, linewidth=0.8, color='#2864b4')
            self.axes.set_xlabel('bag time (s)')
            self.axes.set_ylabel(field)
            self.axes.grid(True, alpha=0.25)
            self.cursor = self.axes.axvline(
                self.current_seconds, color='#d62728', linewidth=1.0,
                animated=True,
            )
            index = self._nearest_index(times, self.current_seconds)
            self.marker, = self.axes.plot(
                [times[index]], [values[index]], 'o', color='#d62728', markersize=4,
                animated=True,
            )
            self._set_value(field, values[index])
        # Field changes and resizes redraw the static trace once. Cursor moves
        # use the cached background below instead of repainting both plots.
        self.canvas.draw()

    @staticmethod
    def _nearest_index(times, seconds):
        index = int(np.searchsorted(times, seconds))
        if index <= 0:
            return 0
        if index >= times.size:
            return times.size - 1
        if seconds - times[index - 1] <= times[index] - seconds:
            return index - 1
        return index

    def _set_value(self, field, value):
        self.value_label.setText(f'{field} = {float(value):.5g}')

    def _on_canvas_draw(self, _event):
        self.plot_background = self.canvas.copy_from_bbox(self.axes.bbox)
        self._blit_cursor()

    def _blit_cursor(self):
        if self.plot_background is None or self.cursor is None or self.marker is None:
            return
        self.canvas.restore_region(self.plot_background)
        self.axes.draw_artist(self.cursor)
        self.axes.draw_artist(self.marker)
        self.canvas.blit(self.axes.bbox)

    def set_time(self, seconds):
        self.current_seconds = seconds
        if self.cursor is None or self.marker is None:
            return
        times = self.telemetry.times[self.topic]
        values = self.telemetry.values[self.topic][self.selector.currentText()]
        index = self._nearest_index(times, seconds)
        self.cursor.set_xdata([seconds, seconds])
        self.marker.set_data([times[index]], [values[index]])
        self._set_value(self.selector.currentText(), values[index])
        self._blit_cursor()


class ReplayWindow(QtWidgets.QMainWindow):
    """Synchronized four-panel Vibe experiment replay window."""

    def __init__(self, bag, telemetry):
        super().__init__()
        self.bag = bag
        self.frame_seeker = TopicSeeker(bag, FRAME_TOPIC)
        self.attention_seeker = TopicSeeker(bag, ATTENTION_TOPIC)
        self.token_seeker = TopicSeeker(bag, TOKENS_TOPIC)
        self.grid = self._read_grid()
        self.latest_frame = None
        self.latest_attention = None
        self.latest_attention_ns = None
        self.latest_query_names = []

        self.setWindowTitle(f'Vibe bag replay — {bag.path.name}')
        self.resize(1450, 950)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        outer = QtWidgets.QVBoxLayout(central)

        header = QtWidgets.QHBoxLayout()
        self.bag_label = QtWidgets.QLabel(str(bag.path))
        self.time_label = QtWidgets.QLabel('0.000 s')
        self.time_label.setMinimumWidth(170)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        duration_ms = max(1, min(2_147_483_647, bag.duration_ns // 1_000_000))
        self.slider.setRange(0, int(duration_ms))
        self.slider.setSingleStep(20)
        self.slider.setPageStep(1000)
        header.addWidget(self.bag_label)
        header.addWidget(self.slider, 1)
        header.addWidget(self.time_label)
        outer.addLayout(header)

        content = QtWidgets.QGridLayout()
        self.frame_panel = ImagePanel(FRAME_TOPIC)
        self.attention_panel = AttentionPanel()
        self.lowstate_panel = TelemetryPanel(
            LOWSTATE_TOPIC, telemetry, 'imu.rpy.pitch',
        )
        self.lowcmd_panel = TelemetryPanel(
            LOWCMD_TOPIC, telemetry, 'left_hip_pitch_joint.q',
        )
        content.addWidget(self.frame_panel, 0, 0)
        content.addWidget(self.attention_panel, 0, 1)
        content.addWidget(self.lowstate_panel, 1, 0)
        content.addWidget(self.lowcmd_panel, 1, 1)
        content.setRowStretch(0, 1)
        content.setRowStretch(1, 1)
        content.setColumnStretch(0, 1)
        content.setColumnStretch(1, 1)
        outer.addLayout(content, 1)

        self.refresh_timer = QtCore.QTimer(self)
        # The recorded visual streams are capped at 50 Hz, so faster refreshes
        # cannot reveal another frame and would only add duplicate bag seeks.
        self.refresh_timer.setInterval(20)
        self.refresh_timer.timeout.connect(self._refresh_visuals)
        self.slider.valueChanged.connect(self._on_time_changed)
        self.attention_panel.selector.currentIndexChanged.connect(
            self._render_attention,
        )
        self._on_time_changed(0)

    def _read_grid(self):
        message, _ = self.token_seeker.nearest(self.bag.start_ns)
        if message and message.grid_h and message.grid_w:
            return int(message.grid_h), int(message.grid_w)
        return None

    def _on_time_changed(self, milliseconds):
        seconds = milliseconds / 1000.0
        self.time_label.setText(
            f'{seconds:.3f} / {self.bag.duration_ns / NANOSECONDS_PER_SECOND:.3f} s',
        )
        self.lowstate_panel.set_time(seconds)
        self.lowcmd_panel.set_time(seconds)
        # Throttle expensive bag seeks instead of debouncing them. Restarting
        # this timer for every mouse event made images wait until dragging
        # stopped; leaving an active timer alone renders the newest cursor
        # position continuously at a bounded rate.
        if not self.refresh_timer.isActive():
            self.refresh_timer.start()

    def _refresh_visuals(self):
        target_ns = self.bag.start_ns + self.slider.value() * 1_000_000
        frame_message, frame_ns = self.frame_seeker.nearest(target_ns)
        self.latest_frame = _image_to_bgr(frame_message)
        frame_status = f'{FRAME_TOPIC} | {_time_delta(frame_ns, target_ns)}'
        if frame_message is not None and self.latest_frame is None:
            frame_status += f' | unsupported encoding {frame_message.encoding}'
        self.frame_panel.set_image(self.latest_frame, frame_status)

        attention, attention_ns = self.attention_seeker.nearest(target_ns)
        self.latest_attention = self._attention_array(attention)
        self.latest_attention_ns = attention_ns
        self.latest_query_names = self._query_names(attention, self.latest_attention)
        self.attention_panel.set_queries(self.latest_query_names)
        self._render_attention()
        if not self.slider.isSliderDown():
            self.refresh_timer.stop()

    @staticmethod
    def _attention_array(message):
        if message is None or len(message.layout.dim) < 2:
            return None
        rows = int(message.layout.dim[0].size)
        columns = int(message.layout.dim[1].size)
        if rows <= 0 or columns <= 0 or len(message.data) < rows * columns:
            return None
        return np.asarray(
            message.data[:rows * columns], dtype=np.float32,
        ).reshape(rows, columns)

    @staticmethod
    def _query_names(message, attention):
        if message is None or attention is None:
            return []
        names = (message.layout.dim[0].label or '').split('|')
        if len(names) != attention.shape[0]:
            names = [f'q{index}' for index in range(attention.shape[0])]
        return names

    def _render_attention(self):
        target_ns = self.bag.start_ns + self.slider.value() * 1_000_000
        status = (
            f'{ATTENTION_TOPIC} | '
            f'{_time_delta(self.latest_attention_ns, target_ns)}'
        )
        if self.latest_frame is None or self.latest_attention is None:
            self.attention_panel.set_image(None, status)
            return
        if self.grid is None:
            self.attention_panel.set_image(None, status + ' | token grid unavailable')
            return

        selection = self.attention_panel.selector.currentIndex() - 1
        if selection < 0:
            values = self.latest_attention.mean(axis=0)
            query = 'mean(all queries)'
        else:
            selection = min(selection, self.latest_attention.shape[0] - 1)
            values = self.latest_attention[selection]
            query = self.latest_query_names[selection]

        grid_h, grid_w = self.grid
        if values.size != grid_h * grid_w:
            self.attention_panel.set_image(
                None,
                status + f' | {values.size} patches, token grid {grid_h}x{grid_w}',
            )
            return
        patch_map = values.reshape(grid_h, grid_w)
        minimum = float(np.min(patch_map))
        maximum = float(np.max(patch_map))
        if maximum > minimum:
            patch_map = (patch_map - minimum) / (maximum - minimum)
        else:
            patch_map = np.zeros_like(patch_map)
        heat = cv2.applyColorMap(
            np.asarray(np.clip(patch_map, 0.0, 1.0) * 255.0, dtype=np.uint8),
            cv2.COLORMAP_JET,
        )
        heat = cv2.resize(
            heat,
            (self.latest_frame.shape[1], self.latest_frame.shape[0]),
            interpolation=cv2.INTER_NEAREST,
        )
        overlay = cv2.addWeighted(self.latest_frame, 0.55, heat, 0.45, 0.0)
        self.attention_panel.set_image(
            overlay, status + f' | {query} | grid {grid_h}x{grid_w}',
        )


def _parse_args():
    parser = argparse.ArgumentParser(
        description='Interactively inspect a recorded Vibe experiment bag.',
    )
    parser.add_argument(
        'bag', nargs='?', help='bag directory; defaults to the newest completed bag',
    )
    parser.add_argument(
        '--bag-root', default=os.environ.get('VIBE_BAG_DIR', '~/unitree_ros2/bags'),
        help='directory searched when BAG is omitted (default: %(default)s)',
    )
    return parser.parse_args()


def main():
    args = _parse_args()
    set_logger_level('rosbag2_storage', LoggingSeverity.WARN)
    try:
        bag_path = _normalize_bag_path(args.bag) if args.bag else _latest_bag(args.bag_root)
        bag = BagAccess(bag_path)
        print(f'replay_vibes: loading telemetry from {bag.path}', file=sys.stderr)
        telemetry = Telemetry(bag)
    except RuntimeError as error:
        print(f'replay_vibes: {error}', file=sys.stderr)
        return 1

    app = QtWidgets.QApplication(sys.argv[:1])
    window = ReplayWindow(bag, telemetry)
    window.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
