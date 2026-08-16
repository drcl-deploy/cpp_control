r"""Closed-loop Repose: the shared Vibe runtime, plus the planner that drives it.

    ros2 launch cpp_control g1_sys1_repose.launch.py \\
        artifact_dir:=/path/to/export [checkpoint:=<training step>]
    ros2 run cpp_control sys1_console.py        # pick the target colour, live

This is a task wrapper like g1_vibe_repose.launch.py, not a layer on top of it:
both include g1_vibe_sonic.launch.py, so the bridge, the encoder and the
controller come up identically and only `sys1:=true` plus this extra node
differ. Open-loop Repose stays exactly what it was.

Prereq beyond the open-loop rig: the camera wire must carry DEPTH —
`camera_streamer.py --depth` on the Orin, or `camera_depth: 1` in the sim
config. sys1 refuses to plan without it and says so.
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
    pkg = get_package_share_directory('cpp_control')
    common = os.path.join(pkg, 'launch', 'g1_vibe_sonic.launch.py')
    default_config = os.path.join(pkg, 'config', 'sys1', 'g1_repose.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('artifact_dir',
                              description='Absolute Repose checkpoint directory'),
        DeclareLaunchArgument('checkpoint', default_value='',
                              description='Artifact step if the directory has multiple exports'),
        DeclareLaunchArgument('sys1_config', default_value=default_config),
        DeclareLaunchArgument('goal_color', default_value='4',
                              description='0 red 1 orange 2 green 3 yellow 4 blue 5 pink; '
                                          'drives the policy one-hot AND the planner ladder'),
        # One address for both halves of the run: the encoder pulls colour from
        # it, the planner pulls colour+depth from it. setup.sh selects it.
        DeclareLaunchArgument(
            'camera_ip',
            default_value=os.environ.get('VIBE_CAMERA_ADDRESS', '127.0.0.1'),
            description='TCP camera source selected by setup.sh/setup_local.sh'),
        DeclareLaunchArgument('enable_bridge', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(common),
            launch_arguments={
                'artifact_dir': LaunchConfiguration('artifact_dir'),
                'checkpoint': LaunchConfiguration('checkpoint'),
                'expected_task_family': 'repose',
                'goal_color': LaunchConfiguration('goal_color'),
                'camera_ip': LaunchConfiguration('camera_ip'),
                'enable_bridge': LaunchConfiguration('enable_bridge'),
                'sys1': 'true',
            }.items(),
        ),
        Node(
            package='cpp_control',
            executable='g1_sys1_repose_node',
            name='g1_sys1_repose_node',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('sys1_config'),
                'camera_host': LaunchConfiguration('camera_ip'),
                # The same argument sys0 gets: one colour, both halves, from
                # boot rather than from the first console keypress.
                'target_color': LaunchConfiguration('goal_color'),
            }],
            # A planner without its controller is a robot holding its last
            # reference. Tear the run down together, like its siblings.
            on_exit=Shutdown(reason='sys1 planner exited'),
        ),
    ])
