"""Launch Dodge on the shared G1 Vibe runtime."""

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
        DeclareLaunchArgument(
            'artifact', default_value='',
            description='Dodge export under $VIBE_ASSET_ROOT/models: <run>[/<export>]'),
        DeclareLaunchArgument(
            'artifact_dir', default_value='',
            description='Dodge export directory; relative resolves under $VIBE_ASSET_ROOT'),
        DeclareLaunchArgument('checkpoint', default_value='',
                              description='Artifact step if the directory has multiple exports'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(common),
            launch_arguments={
                'artifact': LaunchConfiguration('artifact'),
                'artifact_dir': LaunchConfiguration('artifact_dir'),
                'checkpoint': LaunchConfiguration('checkpoint'),
                'expected_task_family': 'dodge',
            }.items(),
        ),
    ])
