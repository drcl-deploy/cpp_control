"""Launch UOLM on the shared G1 Vibe runtime."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    pkg = get_package_share_directory('cpp_control')
    common = os.path.join(pkg, 'launch', 'g1_vibe_sonic.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('artifact_dir',
                              description='Absolute UOLM checkpoint directory'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(common),
            launch_arguments={
                'onnx_path': PathJoinSubstitution([
                    LaunchConfiguration('artifact_dir'), 'model_24999.onnx']),
                'expected_task_family': 'uolm',
            }.items(),
        ),
    ])
