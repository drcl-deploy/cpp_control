"""
Launch the G1 SONIC tracker (exported vibe policy + motion clip).

Usage:
    ros2 launch cpp_control g1_sonic_tracker.launch.py
    ros2 launch cpp_control g1_sonic_tracker.launch.py \
        onnx_path:=/path/to/policy.onnx motion_path:=/path/to/motion.npz
    # hardware: motion_path:='' — boot to stand, stream clips instead:
    ros2 launch cpp_control g1_sonic_tracker.launch.py motion_path:=''
    publish-motion /path/to/motion.npz     # stage; A starts it

The .manifest.json is expected next to the .onnx (exporter default);
override with manifest_path:= if it lives elsewhere.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('cpp_control')
    default_config = os.path.join(pkg, 'config', 'tracker', 'g1_sonic.yaml')

    with open(default_config, 'r') as f:
        config = yaml.safe_load(f)
    default_onnx = os.path.join(pkg, 'models', config['onnx_path'])
    default_motion = os.path.join(pkg, 'models', config['motion_path'])

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=default_config),
        DeclareLaunchArgument('onnx_path', default_value=default_onnx),
        DeclareLaunchArgument('manifest_path', default_value=''),
        DeclareLaunchArgument('motion_path', default_value=default_motion),
        DeclareLaunchArgument('motion_start_frame', default_value='0'),
        DeclareLaunchArgument('il_ordered', default_value='false',
                              description='motion npz is IL-ordered (retargeted dataset)'),

        Node(
            package='cpp_control',
            executable='g1_sonic_node',
            name='g1_sonic_node',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
                'onnx_path': LaunchConfiguration('onnx_path'),
                'manifest_path': LaunchConfiguration('manifest_path'),
                'motion_path': LaunchConfiguration('motion_path'),
                'motion_start_frame': LaunchConfiguration('motion_start_frame'),
                'il_ordered': LaunchConfiguration('il_ordered'),
            }],
        ),
    ])
