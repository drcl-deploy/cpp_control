r"""
Launch the telemetry bridge, vision encoder, and manifest-selected G1 Vibe runtime.

The encoder process starts first, followed by the controller as a separate
process. The controller validates the encoder backbone against the policy's kv
port on the first token.

Usage:
    ros2 launch cpp_control g1_vibe_sonic.launch.py \\
        artifact:=<run>/<export> motion_path:=<clip>.npz
    # hardware: no motion_path — boot to stand, stream clips instead:
    ros2 launch cpp_control g1_vibe_sonic.launch.py artifact:=<run>/<export>
    publish-motion <clip>.npz              # stage; A starts it
    # offboard live camera + attention overlay:
    bash scripts/vibe/stream_vibes.sh

Every path here is relative to $VIBE_ASSET_ROOT (setup.sh: <repo>/vibe), so a
command line is one string on the desktop and on the Orin; absolute still wins.
`artifact:=<run>[/<export>]` names a training run under models/ and finds the
export inside it, whatever the run's own layout; `artifact_dir:=` still takes a
directory. Either way it must hold a matching .onnx/.manifest.json pair, and
multiple pairs need checkpoint:=<training step>.

Prefer one of the task launch files, which selects a packaged artifact and
pins expected_task_family. This generic entry point remains useful for export
development; experiments should use a task wrapper, including Repose.
"""

import glob
import json
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _checkpoint_keys(manifest_path, artifact_stem):
    """Return human-friendly selectors for one exported artifact."""
    keys = {artifact_stem}
    match = re.search(r'(\d+)$', artifact_stem)
    if match:
        keys.add(match.group(1))

    try:
        with open(manifest_path, 'r') as stream:
            checkpoint_path = json.load(stream).get('checkpoint', '')
    except (OSError, ValueError, AttributeError):
        checkpoint_path = ''

    checkpoint_stem = os.path.splitext(os.path.basename(checkpoint_path))[0]
    if checkpoint_stem:
        keys.add(checkpoint_stem)
        match = re.search(r'(\d+)$', checkpoint_stem)
        if match:
            keys.add(match.group(1))
    return keys


def asset_path(rel):
    """Absolute form of `rel`, mirroring C++ common/asset_path.hpp exactly."""
    if not rel:
        return rel
    rel = os.path.expanduser(rel)
    if os.path.isabs(rel):
        return rel
    roots = [root for root in (os.environ.get('VIBE_ASSET_ROOT', ''),
                               os.path.join(get_package_share_directory('cpp_control'),
                                            'models')) if root]
    hits = [os.path.join(root, rel) for root in roots
            if os.path.exists(os.path.join(root, rel))]
    if len(hits) == 1:
        return hits[0]
    if not roots:
        raise RuntimeError(
            "asset path '%s' is relative and no asset root is set. Source "
            'setup.sh, which exports VIBE_ASSET_ROOT, or pass an absolute path.' % rel)
    if not hits:
        raise RuntimeError("asset path '%s' is under none of:\n  %s"
                           % (rel, '\n  '.join(os.path.join(r, rel) for r in roots)))
    raise RuntimeError("asset path '%s' is ambiguous — it exists under:\n  %s"
                       % (rel, '\n  '.join(hits)))


def _export_dirs(run_dir):
    """Directories under a training run that hold an .onnx/.manifest.json pair."""
    found = []
    for depth in ('*', '*/*'):
        for path in sorted(glob.glob(os.path.join(run_dir, depth))):
            if os.path.isdir(path) and _pairs(path):
                found.append(path)
    return found


def _artifact_dir(context):
    """Resolve the export directory from `artifact_dir:=` or `artifact:=`."""
    explicit = LaunchConfiguration('artifact_dir').perform(context).strip()
    shorthand = LaunchConfiguration('artifact').perform(context).strip()
    if explicit and shorthand:
        raise RuntimeError('Pass artifact_dir OR artifact, not both.')
    if explicit:
        return asset_path(explicit)
    if not shorthand:
        raise RuntimeError(
            'Pass artifact:=<run>[/<export>] or artifact_dir:=<path>.')

    run, _, export = shorthand.partition('/')
    run_dir = asset_path(os.path.join('models', run))
    found = _export_dirs(run_dir)
    if export:
        found = [path for path in found if os.path.basename(path) == export]
    if len(found) == 1:
        return found[0]
    if not found:
        raise RuntimeError("No export under '%s' matching artifact:=%s"
                           % (run_dir, shorthand))
    raise RuntimeError('artifact:=%s is ambiguous. Name one of: %s'
                       % (shorthand, ', '.join(os.path.basename(p) for p in found)))


def _pairs(artifact_dir):
    """Return the (.onnx, .manifest.json, stem) triples sitting in a directory."""
    suffix = '.manifest.json'
    found = []
    for manifest_path in sorted(glob.glob(os.path.join(artifact_dir, '*' + suffix))):
        artifact_stem = os.path.basename(manifest_path)[:-len(suffix)]
        onnx_path = os.path.join(artifact_dir, artifact_stem + '.onnx')
        if os.path.isfile(onnx_path):
            found.append((onnx_path, manifest_path, artifact_stem))
    return found


def _resolve_artifact(context):
    artifact_dir = _artifact_dir(context)
    checkpoint = LaunchConfiguration('checkpoint').perform(context).strip()

    if not os.path.isdir(artifact_dir):
        raise RuntimeError('Vibe artifact_dir is not a directory: ' + artifact_dir)

    pairs = _pairs(artifact_dir)
    if not pairs:
        raise RuntimeError(
            'Vibe artifact_dir has no matching .onnx/.manifest.json pair: ' +
            artifact_dir)

    if checkpoint:
        pairs = [pair for pair in pairs
                 if checkpoint in _checkpoint_keys(pair[1], pair[2])]
        if not pairs:
            raise RuntimeError(
                "No Vibe artifact matches checkpoint '%s' in %s" %
                (checkpoint, artifact_dir))

    if len(pairs) != 1:
        candidates = ', '.join(pair[2] for pair in pairs)
        raise RuntimeError(
            'Vibe artifact_dir is ambiguous (%s). Pass checkpoint:=<step>.' %
            candidates)

    return pairs[0][0], pairs[0][1]


def _launch_runtime(context, encoder_pkg):
    onnx_path, manifest_path = _resolve_artifact(context)
    motion_path = asset_path(LaunchConfiguration('motion_path').perform(context).strip())
    encoder_config = os.path.join(encoder_pkg, 'config', 'encoder.yaml')

    encoder = Node(
        package='vision_encoders',
        executable='encoder_node',
        name='encoder_node',
        output='screen',
        parameters=[encoder_config, {
            'model': LaunchConfiguration('encoder_tag'),
            'camera_ip': LaunchConfiguration('camera_ip'),
        }],
        on_exit=Shutdown(reason='Vibe encoder exited'),
    )

    controller = Node(
        package='cpp_control',
        executable='g1_vibe_sonic_node',
        name='g1_vibe_sonic_node',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'onnx_path': onnx_path,
            'manifest_path': manifest_path,
            'motion_path': motion_path,
            'motion_start_frame': LaunchConfiguration('motion_start_frame'),
            'il_ordered': LaunchConfiguration('il_ordered'),
            'tokens_topic': LaunchConfiguration('tokens_topic'),
            'reference_topic': LaunchConfiguration('reference_topic'),
            'encoder_tag': LaunchConfiguration('encoder_tag'),
            'expected_task_family': LaunchConfiguration('expected_task_family'),
            'goal_color': LaunchConfiguration('goal_color'),
            'prep_joints': LaunchConfiguration('prep_joints'),
            'prep_rate': LaunchConfiguration('prep_rate'),
            'prep_min_s': LaunchConfiguration('prep_min_s'),
            'prep_max_s': LaunchConfiguration('prep_max_s'),
            'planner': LaunchConfiguration('planner'),
            'calibration_lock': LaunchConfiguration('calibration_lock'),
            'status_rate_hz': LaunchConfiguration('status_rate_hz'),
            'stand_yaw_teleop': LaunchConfiguration('stand_yaw_teleop'),
            'stand_yaw_rate_deg_s': LaunchConfiguration('stand_yaw_rate_deg_s'),
            'stand_yaw_accel_deg_s2': LaunchConfiguration('stand_yaw_accel_deg_s2'),
            'stand_yaw_deadband': LaunchConfiguration('stand_yaw_deadband'),
            'stand_yaw_timeout_s': LaunchConfiguration('stand_yaw_timeout_s'),
            'stand_yaw_settle_rate_deg_s': LaunchConfiguration(
                'stand_yaw_settle_rate_deg_s'),
            'stand_yaw_settle_gyro_deg_s': LaunchConfiguration(
                'stand_yaw_settle_gyro_deg_s'),
            'stand_yaw_settle_error_deg': LaunchConfiguration(
                'stand_yaw_settle_error_deg'),
            'stand_walk_teleop': LaunchConfiguration('stand_walk_teleop'),
            'stand_walk_motion': LaunchConfiguration('stand_walk_motion'),
            'stand_walk_distance_m': LaunchConfiguration('stand_walk_distance_m'),
            'stand_walk_lead_s': LaunchConfiguration('stand_walk_lead_s'),
            'stand_walk_exit_s': LaunchConfiguration('stand_walk_exit_s'),
            'stand_walk_pause_s': LaunchConfiguration('stand_walk_pause_s'),
            'stand_walk_deadband': LaunchConfiguration('stand_walk_deadband'),
            'stand_walk_timeout_s': LaunchConfiguration('stand_walk_timeout_s'),
        }],
        on_exit=Shutdown(reason='Vibe controller exited'),
    )

    bridge = Node(
        package='cdr_tcp_bridge',
        executable='cdr_tcp_bridge',
        name='cdr_tcp_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_bridge')),
        parameters=[{
            'role': 'server',
            'bind_address': LaunchConfiguration('bridge_address'),
            'config_path': LaunchConfiguration('bridge_config'),
        }],
        on_exit=Shutdown(reason='Vibe telemetry bridge exited'),
    )

    return [
        RegisterEventHandler(
            OnProcessStart(
                target_action=encoder,
                on_start=[controller],
            ),
        ),
        bridge,
        encoder,
    ]


def generate_launch_description():
    pkg = get_package_share_directory('cpp_control')
    encoder_pkg = get_package_share_directory('vision_encoders')
    default_config = os.path.join(pkg, 'config', 'vibe', 'g1_vibe_sonic.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=default_config),
        DeclareLaunchArgument(
            'artifact', default_value='',
            description='shorthand for an export under $VIBE_ASSET_ROOT/models: '
                        '<run>[/<export>], e.g. g1_repose_big_cube_floor/011pgzbh'),
        DeclareLaunchArgument(
            'artifact_dir', default_value='',
            description='export directory holding a matching ONNX/manifest pair; '
                        'relative resolves under $VIBE_ASSET_ROOT'),
        DeclareLaunchArgument(
            'checkpoint', default_value='',
            description='Training step selector; required only for ambiguous directories'),
        DeclareLaunchArgument('motion_path', default_value='',
                              description='npz clip, relative to $VIBE_ASSET_ROOT; '
                                          'empty = stand until streamed'),
        DeclareLaunchArgument('motion_start_frame', default_value='0'),
        DeclareLaunchArgument('il_ordered', default_value='true',
                              description='motion npz is IL-ordered (retargeted dataset)'),
        DeclareLaunchArgument('tokens_topic', default_value='/enc/tokens'),
        DeclareLaunchArgument('reference_topic', default_value='/tracker/reference'),
        DeclareLaunchArgument('encoder_tag', default_value='theia-tiny'),
        DeclareLaunchArgument(
            'camera_ip',
            default_value=os.environ.get('VIBE_CAMERA_ADDRESS', '127.0.0.1'),
            description='TCP camera source selected by setup.sh/setup_local.sh'),
        DeclareLaunchArgument(
            'bridge_address',
            default_value=os.environ.get('VIBE_BRIDGE_ADDRESS', 'gilfoyle-rth'),
            description='CDR/TCP bind address selected by setup.sh/setup_local.sh'),
        DeclareLaunchArgument(
            'bridge_config',
            default_value=os.path.join(
                get_package_share_directory('cdr_tcp_bridge'),
                'config', 'full.yaml'),
            description='CDR/TCP channel profile used by server and client'),
        DeclareLaunchArgument(
            'enable_bridge', default_value='true',
            description='start telemetry server (disable only for no-telemetry benchmarks)'),
        DeclareLaunchArgument('expected_task_family', default_value=''),
        DeclareLaunchArgument('goal_color', default_value='0',
                              description='goal up-face color index (one-hot 6)'),
        DeclareLaunchArgument('prep_joints', default_value='arms',
                              description='joints the L1 lead-in moves: arms | arms_waist | all'),
        DeclareLaunchArgument('prep_rate', default_value='1.5',
                              description='L1 lead-in speed, rad/s (max|dq| sets duration)'),
        DeclareLaunchArgument('prep_min_s', default_value='0.5'),
        DeclareLaunchArgument('prep_max_s', default_value='2.0'),
        DeclareLaunchArgument('planner', default_value='false',
                              description='A arms planner auto-commit; RB disarms to '
                                          'stand; prep gate off, status published'),
        DeclareLaunchArgument(
            'calibration_lock', default_value='false',
            description='hold nominal SONIC stand and refuse every motion source'),
        DeclareLaunchArgument(
            'status_rate_hz', default_value='20.0',
            description='ControllerStatus rate with planner/calibration active'),
        DeclareLaunchArgument(
            'stand_yaw_teleop', default_value='false',
            description='right-stick X controls heading while SONIC holds stand'),
        DeclareLaunchArgument('stand_yaw_rate_deg_s', default_value='45.0'),
        DeclareLaunchArgument(
            'stand_yaw_accel_deg_s2', default_value='180.0'),
        DeclareLaunchArgument('stand_yaw_deadband', default_value='0.15'),
        DeclareLaunchArgument('stand_yaw_timeout_s', default_value='0.25'),
        DeclareLaunchArgument('stand_yaw_settle_rate_deg_s', default_value='2.0'),
        DeclareLaunchArgument('stand_yaw_settle_gyro_deg_s', default_value='5.0'),
        DeclareLaunchArgument('stand_yaw_settle_error_deg', default_value='5.0'),
        DeclareLaunchArgument(
            'stand_walk_teleop', default_value='false',
            description='left-stick forward runs bounded walk bouts in planner stand'),
        DeclareLaunchArgument('stand_walk_motion', default_value=''),
        DeclareLaunchArgument('stand_walk_distance_m', default_value='0.32'),
        DeclareLaunchArgument('stand_walk_lead_s', default_value='0.35'),
        DeclareLaunchArgument('stand_walk_exit_s', default_value='0.45'),
        DeclareLaunchArgument('stand_walk_pause_s', default_value='0.35'),
        DeclareLaunchArgument('stand_walk_deadband', default_value='0.20'),
        DeclareLaunchArgument('stand_walk_timeout_s', default_value='0.25'),

        OpaqueFunction(function=_launch_runtime, args=[encoder_pkg]),
    ])
