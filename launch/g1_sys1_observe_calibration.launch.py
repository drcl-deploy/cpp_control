r"""
Hold nominal stand and publish paired RGB-D for Sys1 observation calibration.

The controller remains the only low-level command source. `calibration_lock`
prevents every motion/reference input from replacing its nominal SONIC stand;
zeroing and damping controls remain authoritative. The observe node only reads
camera/state and publishes telemetry.

Sim/hardware:
    ros2 launch cpp_control g1_sys1_observe_calibration.launch.py \
        artifact_dir:=/path/to/repose/export [checkpoint:=STEP]

Offboard:
    bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/sys1/stream_sys1_observe.sh \
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
    sys1_config = os.path.join(cpp, 'config', 'sys1', 'g1_repose.yaml')
    bridge_config = os.path.join(bridge, 'config', 'sys1_observe.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'artifact_dir',
            description='Absolute Repose checkpoint/export directory'),
        DeclareLaunchArgument('checkpoint', default_value=''),
        DeclareLaunchArgument('sys1_config', default_value=sys1_config),
        DeclareLaunchArgument(
            'camera_ip',
            default_value=os.environ.get('VIBE_CAMERA_ADDRESS', '127.0.0.1')),
        DeclareLaunchArgument('enable_bridge', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(common),
            launch_arguments={
                'artifact_dir': LaunchConfiguration('artifact_dir'),
                'checkpoint': LaunchConfiguration('checkpoint'),
                'expected_task_family': 'repose',
                'motion_path': '',
                'camera_ip': LaunchConfiguration('camera_ip'),
                'enable_bridge': LaunchConfiguration('enable_bridge'),
                'bridge_config': bridge_config,
                'sys1': 'false',
                'calibration_lock': 'true',
                'status_rate_hz': '50.0',
            }.items(),
        ),
        Node(
            package='cpp_control',
            executable='g1_sys1_observe_node',
            name='g1_sys1_observe_node',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('sys1_config'),
                'camera_host': LaunchConfiguration('camera_ip'),
            }],
            on_exit=Shutdown(reason='Sys1 observe publisher exited'),
        ),
    ])
