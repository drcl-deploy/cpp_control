"""
Launch G1 Visual Backbone Policy — Residual Action.

Runs two ONNX policies (HLC + WBC) with visual embedding input from Theia.

Usage:
    ros2 launch cpp_control g1_visual_backbone_residual.launch.py
    ros2 launch cpp_control g1_visual_backbone_residual.launch.py onnx_model_path:=/path/to/hlc.onnx
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

    default_config = os.path.join(pkg, 'config', 'visual_backbone_policies', 'g1_residual.yaml')

    # Read ONNX paths from config
    with open(default_config, 'r') as f:
        config = yaml.safe_load(f)

    hlc_model = config.get('onnx_path', '')
    if hlc_model:
        hlc_model = os.path.join(pkg, 'models', hlc_model)

    wbc_model = config.get('wbc_onnx_path', '')
    if wbc_model:
        wbc_model = os.path.join(pkg, 'models', wbc_model)

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=default_config),
        DeclareLaunchArgument('onnx_model_path', default_value=hlc_model,
                              description='HLC policy ONNX'),
        DeclareLaunchArgument('wbc_onnx_model_path', default_value=wbc_model,
                              description='WBC policy ONNX (textop)'),
        DeclareLaunchArgument('embedding_topic', default_value='/theia_embedding'),
        DeclareLaunchArgument('object_goal_topic', default_value='/object_goal'),
        DeclareLaunchArgument('motion_topic', default_value='/tracker/motion'),

        Node(
            package='cpp_control',
            executable='g1_vbp_residual_node',
            name='g1_vbp_residual_node',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
                'onnx_model_path': LaunchConfiguration('onnx_model_path'),
                'wbc_onnx_model_path': LaunchConfiguration('wbc_onnx_model_path'),
                'embedding_topic': LaunchConfiguration('embedding_topic'),
                'object_goal_topic': LaunchConfiguration('object_goal_topic'),
                'motion_topic': LaunchConfiguration('motion_topic'),
            }],
        ),
    ])
