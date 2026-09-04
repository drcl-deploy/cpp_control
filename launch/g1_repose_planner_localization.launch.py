r"""
Inert v8 localization check: hold nominal stand and publish continuous RGB-D.

This launch never starts the repose planner. The shared controller runs with
calibration_lock=true, so reference motion cannot replace the nominal SONIC
stand; zeroing and damping remain authoritative.

Onboard:
    ros2 launch cpp_control g1_repose_planner_localization.launch.py \
        artifact:=<run>/<export> env:=real

Offboard (live viewer plus a continuous, replayable bag):
    ros2 run cpp_control repose_check_observe.sh record
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    cpp = get_package_share_directory('cpp_control')
    calibration = os.path.join(
        cpp, 'launch', 'g1_repose_planner_calibration.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'artifact', default_value='',
            description='Repose export under $VIBE_ASSET_ROOT/models'),
        DeclareLaunchArgument('artifact_dir', default_value=''),
        DeclareLaunchArgument('checkpoint', default_value=''),
        DeclareLaunchArgument(
            'env', default_value='real', choices=['sim', 'real'],
            description='select g1_real_v8.yaml or g1_sim_v8.yaml'),
        DeclareLaunchArgument(
            'planner_config',
            default_value=[os.path.join(
                cpp, 'config', 'repose_planner', 'g1_'),
                LaunchConfiguration('env'), '_v8.yaml']),
        DeclareLaunchArgument(
            'camera_ip',
            default_value=os.environ.get('VIBE_CAMERA_ADDRESS', '127.0.0.1')),
        DeclareLaunchArgument('enable_bridge', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(calibration),
            launch_arguments={
                'artifact': LaunchConfiguration('artifact'),
                'artifact_dir': LaunchConfiguration('artifact_dir'),
                'checkpoint': LaunchConfiguration('checkpoint'),
                'env': LaunchConfiguration('env'),
                'planner_config': LaunchConfiguration('planner_config'),
                'camera_ip': LaunchConfiguration('camera_ip'),
                'enable_bridge': LaunchConfiguration('enable_bridge'),
            }.items(),
        ),
    ])
