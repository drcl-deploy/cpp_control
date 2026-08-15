r"""Launch the manifest-selected G1 Vibe adapt-SONIC runtime.

Prereqs: an encoder publishing /enc/tokens (vision_encoders encoder.launch.py)
whose backbone matches the policy's kv port — validated at first token.

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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('cpp_control')
    default_config = os.path.join(pkg, 'config', 'vibe', 'g1_vibe_sonic.yaml')

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
        DeclareLaunchArgument('expected_task_family', default_value=''),
        DeclareLaunchArgument('goal_color', default_value='0',
                              description='goal up-face color index (one-hot 6)'),
        DeclareLaunchArgument('prep_joints', default_value='arms',
                              description='joints the L1 lead-in moves: arms | arms_waist | all'),
        DeclareLaunchArgument('prep_rate', default_value='1.5',
                              description='L1 lead-in speed, rad/s (max|dq| sets duration)'),
        DeclareLaunchArgument('prep_min_s', default_value='0.5'),
        DeclareLaunchArgument('prep_max_s', default_value='2.0'),

        Node(
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
        ),
    ])
