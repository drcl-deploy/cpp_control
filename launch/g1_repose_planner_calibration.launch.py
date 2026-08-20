r"""
Hold nominal stand and publish paired RGB-D for The planner observation calibration.

The controller remains the only low-level command source. `calibration_lock`
prevents every motion/reference input from replacing its nominal SONIC stand;
zeroing and damping controls remain authoritative. The observe node only reads
camera/state and publishes telemetry.

Sim/hardware:
    ros2 launch cpp_control g1_repose_planner_calibration.launch.py \
        artifact:=<run>/<export> [env:=sim|real]

Offboard:
    bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/planners/repose/repose_stream_observe.sh \
        [record]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            Shutdown)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cpp = get_package_share_directory('cpp_control')
    bridge = get_package_share_directory('cdr_tcp_bridge')
    common = os.path.join(cpp, 'launch', 'g1_vibe_sonic.launch.py')
    bridge_config = os.path.join(bridge, 'config', 'repose_calibration.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'artifact', default_value='',
            description='Repose export under $VIBE_ASSET_ROOT/models: <run>[/<export>]'),
        DeclareLaunchArgument(
            'artifact_dir', default_value='',
            description='Repose export directory; relative resolves under $VIBE_ASSET_ROOT'),
        DeclareLaunchArgument('checkpoint', default_value=''),
        DeclareLaunchArgument('env', default_value='sim',
                              choices=['sim', 'real'],
                              description='which observe block to load'),
        DeclareLaunchArgument(
            'planner_config',
            default_value=[os.path.join(cpp, 'config', 'repose_planner', 'g1_'),
                           LaunchConfiguration('env'), '.yaml']),
        DeclareLaunchArgument(
            'camera_ip',
            default_value=os.environ.get('VIBE_CAMERA_ADDRESS', '127.0.0.1')),
        DeclareLaunchArgument('enable_bridge', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(common),
            launch_arguments={
                'artifact': LaunchConfiguration('artifact'),
                'artifact_dir': LaunchConfiguration('artifact_dir'),
                'checkpoint': LaunchConfiguration('checkpoint'),
                'expected_task_family': 'repose',
                'motion_path': '',
                'camera_ip': LaunchConfiguration('camera_ip'),
                'enable_bridge': LaunchConfiguration('enable_bridge'),
                'bridge_config': bridge_config,
                'planner': 'false',
                'calibration_lock': 'true',
                'status_rate_hz': '50.0',
            }.items(),
        ),
        Node(
            package='cpp_control',
            executable='g1_repose_planner_observe_node',
            name='g1_repose_planner_observe_node',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('planner_config'),
                'camera_host': LaunchConfiguration('camera_ip'),
            }],
            on_exit=Shutdown(reason='repose observe publisher exited'),
        ),
    ])
