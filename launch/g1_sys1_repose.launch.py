r"""Closed-loop Repose: the planner + the controller it drives.

    ros2 launch cpp_control g1_sys1_repose.launch.py artifact:=/path/model.onnx
    ros2 run cpp_control sys1_console.py        # pick the target colour, live

Prereqs: an encoder on /enc/tokens, and a camera wire carrying DEPTH —
`camera_streamer.py --depth` on the Orin, or `camera_depth: 1` in the sim
config. sys1 refuses to plan without it and says so.

The controller half is `g1_vibe_repose.launch.py` with sys1:=true; launched on
its own it is the open-loop rig, unchanged.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('cpp_control')
    controller = os.path.join(pkg, 'launch', 'g1_vibe_repose.launch.py')
    default_config = os.path.join(pkg, 'config', 'sys1', 'g1_repose.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('artifact',
                              description='Absolute path to the Repose policy .onnx'),
        DeclareLaunchArgument('sys1_config', default_value=default_config),
        DeclareLaunchArgument('goal_color', default_value='4'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(controller),
            launch_arguments={
                'artifact': LaunchConfiguration('artifact'),
                'goal_color': LaunchConfiguration('goal_color'),
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
            }],
        ),
    ])
