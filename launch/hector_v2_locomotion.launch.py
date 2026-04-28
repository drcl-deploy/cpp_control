"""
Launch HectorV2 Locomotion Controller.

Usage:
    ros2 launch cpp_control hector_v2_locomotion.launch.py
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

    default_config = os.path.join(pkg, 'config', 'locomotion', 'hector_v2.yaml')

    with open(default_config, 'r') as f:
        config = yaml.safe_load(f)
        default_model = config.get('onnx_path')
        if default_model:
            default_model = os.path.join(pkg, 'models', default_model)
            print(f"Default ONNX model path: {default_model}")
        else:
            raise ValueError("ONNX model path not found in config file")

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=default_config),
        DeclareLaunchArgument('onnx_model_path', default_value=default_model),

        Node(
            package='cpp_control',
            executable='hector_v2_locomotion_node',
            name='hector_v2_locomotion_node',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
                'onnx_model_path': LaunchConfiguration('onnx_model_path'),
            }],
        ),
    ])
