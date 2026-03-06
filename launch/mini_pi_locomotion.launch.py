"""
Launch Mini Pi Locomotion Controller.

Usage:
    ros2 launch cpp_control mini_pi_locomotion.launch.py
    ros2 launch cpp_control mini_pi_locomotion.launch.py onnx_model_path:=/path/to/model.onnx
"""

import os
import platform
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('cpp_control')

    default_config = os.path.join(pkg, 'config', 'locomotion', 'mini_pi.yaml')
    default_model = os.path.join(pkg, 'models','locomotion', 'mini_pi_sim2real.onnx')

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=default_config),
        DeclareLaunchArgument('onnx_model_path', default_value=default_model),

        Node(
            package='cpp_control',
            executable='mini_pi_locomotion_node',
            name='mini_pi_locomotion_node',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
                'onnx_model_path': LaunchConfiguration('onnx_model_path'),
            }],
        ),
    ])
