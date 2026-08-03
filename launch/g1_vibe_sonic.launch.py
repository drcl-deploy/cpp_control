"""
Launch the G1 adapt-SONIC node (vision-adapted exported vibe policy).

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
        DeclareLaunchArgument('goal_color', default_value='0',
                              description='goal up-face color index (one-hot 6)'),

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
                'goal_color': LaunchConfiguration('goal_color'),
            }],
        ),
    ])
