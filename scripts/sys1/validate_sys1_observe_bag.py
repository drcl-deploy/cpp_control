#!/usr/bin/env python3
"""Validate a staged Sys1 RGB-D calibration rosbag."""

import argparse
from collections import Counter, defaultdict
from pathlib import Path

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import yaml


COLOR_NAMES = ('red', 'orange', 'green', 'yellow', 'blue', 'pink')
STATE = '/vibe/sys1/calibration/state'
LIVE_OBS = '/vibe/sys1/observe/observation'
LIVE_IMAGES = (
    '/vibe/sys1/observe/color/compressed',
    '/vibe/sys1/observe/depth',
    '/vibe/sys1/observe/camera_info',
    '/vibe/sys1/observe/labels',
)
SNAPSHOT_OBS = '/vibe/sys1/calibration/observation'
SNAPSHOT_IMAGES = (
    '/vibe/sys1/calibration/color/compressed',
    '/vibe/sys1/calibration/depth',
    '/vibe/sys1/calibration/camera_info',
    '/vibe/sys1/calibration/labels',
)


def _stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def _metadata(path):
    with (path / 'metadata.yaml').open('r', encoding='utf-8') as stream:
        root = yaml.safe_load(stream)
    return root.get('rosbag2_bagfile_information', root)


def _reader(path):
    info = _metadata(path)
    storage_id = str(info.get('storage_identifier', 'sqlite3'))
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'),
    )
    return reader


def validate(path):
    reader = _reader(path)
    topic_types = {item.name: item.type
                   for item in reader.get_all_topics_and_types()}
    snapshot_required = {STATE, SNAPSHOT_OBS, *SNAPSHOT_IMAGES}
    if snapshot_required <= set(topic_types):
        observation_topic = SNAPSHOT_OBS
        image_topics = SNAPSHOT_IMAGES
        require_negative = True
    else:
        observation_topic = LIVE_OBS
        image_topics = LIVE_IMAGES
        require_negative = False
    required = {STATE, observation_topic, *image_topics}
    missing_topics = sorted(required - set(topic_types))
    if missing_topics:
        print('missing topics: ' + ', '.join(missing_topics))
        return 1

    classes = {topic: get_message(topic_types[topic]) for topic in required}
    image_stamps = {topic: set() for topic in image_topics}
    observations = {}
    captured = []
    complete_seen = False

    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic not in required:
            continue
        message = deserialize_message(serialized, classes[topic])
        if topic in image_stamps:
            image_stamps[topic].add(_stamp_ns(message.header))
        elif topic == observation_topic:
            observations[int(message.sequence)] = message
        elif topic == STATE:
            complete_seen = complete_seen or bool(message.complete)
            if message.capturing:
                captured.append(message)

    if not captured:
        print('no selected calibration states; did the operator press LT/Space?')
        return 1

    final_attempt = defaultdict(int)
    for state in captured:
        final_attempt[int(state.expected_color)] = max(
            final_attempt[int(state.expected_color)], int(state.attempt))
    selected = [state for state in captured
                if int(state.attempt) == final_attempt[int(state.expected_color)]]

    errors = []
    by_color = Counter()
    reasons = defaultdict(Counter)
    pose_ok = Counter()
    confusion = [[0 for _ in COLOR_NAMES] for _ in COLOR_NAMES]
    color_miss = Counter()
    negative_observed = Counter()
    negative_false_positive = 0
    sample_indices = defaultdict(list)
    targets = {}
    seen_sequences = set()

    for state in selected:
        truth = int(state.expected_color)
        if truth < -1 or truth >= len(COLOR_NAMES):
            errors.append(f'invalid expected_color {truth}')
            continue
        target = int(state.samples_per_color)
        if target <= 0:
            errors.append(f'expected_color {truth}: invalid target {target}')
        elif truth in targets and targets[truth] != target:
            errors.append(
                f'expected_color {truth}: inconsistent targets '
                f'{targets[truth]} and {target}')
        else:
            targets[truth] = target
        by_color[truth] += 1
        sample_indices[truth].append(int(state.sample_index))
        sequence = int(state.source_sequence)
        if sequence in seen_sequences:
            errors.append(f'sequence {sequence}: duplicate captured label')
        seen_sequences.add(sequence)
        observation = observations.get(sequence)
        if observation is None:
            errors.append(f'sequence {state.source_sequence}: missing observation')
            continue
        stamp = _stamp_ns(state.header)
        if _stamp_ns(observation.header) != stamp:
            errors.append(f'sequence {state.source_sequence}: state/observation stamp mismatch')
        for topic, stamps in image_stamps.items():
            if stamp not in stamps:
                errors.append(f'sequence {state.source_sequence}: missing {topic}')
        if not observation.state_ready or not observation.stand_locked:
            errors.append(f'sequence {state.source_sequence}: acquisition interlock false')
        reasons[truth][observation.reason] += 1
        if observation.pose_ok:
            pose_ok[truth] += 1
        observed_color = int(observation.color)
        if truth == -1:
            if observation.color_ok:
                negative_false_positive += 1
                negative_observed[observed_color] += 1
        elif observation.color_ok and 0 <= observed_color < len(COLOR_NAMES):
            confusion[truth][observed_color] += 1
        else:
            color_miss[truth] += 1

    if not complete_seen:
        errors.append('no complete=true calibration state')
    print(f'bag: {path}')
    print('format: ' + (
        'clicked snapshots' if require_negative
        else 'legacy continuous stream'))
    print('\ncoverage')
    for color, name in enumerate(COLOR_NAMES):
        total = by_color[color]
        pose = pose_ok[color]
        requested = targets.get(color, 0)
        print(f'  {name:7s} {total:5d}  pose_ok {pose:5d} '
              f'({100.0 * pose / max(total, 1):5.1f}%)  '
              f'color_miss {color_miss[color]:5d}')
        if requested <= 0:
            errors.append(f'{name}: no labeled samples')
            continue
        if total != requested:
            errors.append(f'{name}: got {total}, expected {requested}')
        expected_indices = list(range(1, requested + 1))
        if sorted(sample_indices[color]) != expected_indices:
            errors.append(f'{name}: sample_index is not exactly 1..{requested}')

    if require_negative:
        total = by_color[-1]
        requested = targets.get(-1, 0)
        rate = 100.0 * negative_false_positive / max(total, 1)
        print(f'  {"negative":7s} {total:5d}  false positive '
              f'{negative_false_positive:5d} ({rate:5.1f}%)')
        if requested <= 0:
            errors.append('negative: no labeled samples')
        else:
            if total != requested:
                errors.append(
                    f'negative: got {total}, expected {requested}')
            expected_indices = list(range(1, requested + 1))
            if sorted(sample_indices[-1]) != expected_indices:
                errors.append(
                    f'negative: sample_index is not exactly 1..{requested}')

    print('\ncolor confusion (rows=true, columns=observed; misses excluded)')
    print('          ' + ' '.join(f'{name[:3]:>5s}' for name in COLOR_NAMES))
    for color, name in enumerate(COLOR_NAMES):
        print(f'  {name[:7]:7s} ' + ' '.join(
            f'{value:5d}' for value in confusion[color]))

    if require_negative:
        observed_text = ', '.join(
            f'{COLOR_NAMES[color] if 0 <= color < len(COLOR_NAMES) else color}'
            f'={count}' for color, count in negative_observed.most_common())
        print('\nnegative false-positive colors')
        print('  ' + (observed_text or '--'))

    print('\nrejection/result reasons')
    for color, name in enumerate(COLOR_NAMES):
        text = ', '.join(f'{key}={value}'
                         for key, value in reasons[color].most_common())
        print(f'  {name:7s} {text or "--"}')
    if require_negative:
        text = ', '.join(f'{key}={value}'
                         for key, value in reasons[-1].most_common())
        print(f'  {"negative":7s} {text or "--"}')

    if errors:
        print(f'\nFAIL: {len(errors)} integrity error(s)')
        for error in errors[:30]:
            print('  ' + error)
        if len(errors) > 30:
            print(f'  ... {len(errors) - 30} more')
        return 1
    print('\nPASS: complete labeled RGB-D corpus with intact interlocks')
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    args = parser.parse_args()
    path = args.bag.expanduser().resolve()
    if not (path / 'metadata.yaml').is_file():
        parser.error(f'not a completed rosbag directory: {path}')
    raise SystemExit(validate(path))


if __name__ == '__main__':
    main()
