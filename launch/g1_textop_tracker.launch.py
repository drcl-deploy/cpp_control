"""
Launch G1 Tracker Controller (TextOp).

Usage:
    ros2 launch cpp_control g1_textop_tracker.launch.py
    ros2 launch cpp_control g1_textop_tracker.launch.py onnx_model_path:=/path/to/model.onnx
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    pkg = get_package_share_directory('cpp_control')

    default_config = os.path.join(pkg, 'config', 'tracker', 'g1_textop.yaml')

    # Read the ONNX model path from the config file
    with open(default_config, 'r') as f:
        config = yaml.safe_load(f)
        default_model = config.get('onnx_path')
        if default_model:
            default_model = os.path.join(pkg, 'models', default_model)
        else:
            raise ValueError("ONNX model path not found in config file")

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=default_config),
        DeclareLaunchArgument('onnx_model_path', default_value=default_model),
        DeclareLaunchArgument('motion_topic', default_value='/tracker/motion'),

        Node(
            package='cpp_control',
            executable='g1_textop_node',
            name='g1_textop_node',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
                'onnx_model_path': LaunchConfiguration('onnx_model_path'),
                'motion_topic': LaunchConfiguration('motion_topic'),
            }],
        ),
    ])
