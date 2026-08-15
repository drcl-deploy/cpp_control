r"""
Launch the telemetry bridge, vision encoder, and manifest-selected G1 Vibe runtime.

The encoder process starts first, followed by the controller as a separate
process. The controller validates the encoder backbone against the policy's kv
port on the first token.

Usage:
    ros2 launch cpp_control g1_vibe_sonic.launch.py \\
        onnx_path:=/path/to/model.onnx motion_path:=/path/to/motion.npz
    # hardware: no motion_path — boot to stand, stream clips instead:
    ros2 launch cpp_control g1_vibe_sonic.launch.py onnx_path:=/path/to/model.onnx
    publish-motion /path/to/motion.npz     # stage; A starts it
    # smoke the attention rows:
    ros2 run cpp_control attn_viewer.py

The .manifest.json is expected next to the .onnx (exporter default);
override with manifest_path:= if it lives elsewhere.

Prefer one of the task launch files, which selects a packaged artifact and
pins expected_task_family. This generic entry point remains useful for export
development and Repose compatibility.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('cpp_control')
    encoder_pkg = get_package_share_directory('vision_encoders')
    default_config = os.path.join(pkg, 'config', 'vibe', 'g1_vibe_sonic.yaml')
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
            'onnx_path': LaunchConfiguration('onnx_path'),
            'manifest_path': LaunchConfiguration('manifest_path'),
            'motion_path': LaunchConfiguration('motion_path'),
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
        }],
        on_exit=Shutdown(reason='Vibe telemetry bridge exited'),
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=default_config),
        DeclareLaunchArgument('onnx_path'),
        DeclareLaunchArgument('manifest_path', default_value=''),
        DeclareLaunchArgument('motion_path', default_value='',
                              description='npz clip; empty = stand until streamed'),
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

        RegisterEventHandler(
            OnProcessStart(
                target_action=encoder,
                on_start=[controller],
            ),
        ),
        bridge,
        encoder,
    ])
