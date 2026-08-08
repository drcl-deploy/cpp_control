"""Launch UOLM on the shared G1 Vibe runtime."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('cpp_control')
    common = os.path.join(pkg, 'launch', 'g1_vibe_sonic.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('onnx_path',
                              description='Local UOLM ONNX export'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(common),
            launch_arguments={
                'onnx_path': LaunchConfiguration('onnx_path'),
                'expected_task_family': 'uolm',
            }.items(),
        ),
    ])
