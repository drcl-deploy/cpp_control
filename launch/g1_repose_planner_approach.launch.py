r"""Safe v8.5 approach-only smoke test.

Starts the normal Repose controller and planner, but suppresses every flip.
With the robot in POLICY, drag the cube: a winning entry residual above the
configured gate may produce TURN -> APPROACH -> SETTLE -> fresh observation.
Inside the gate the robot keeps settling. RB remains the immediate disarm.

    ros2 launch cpp_control g1_repose_planner_approach.launch.py \
      artifact_dir:=/absolute/export env:=sim
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    cpp = get_package_share_directory('cpp_control')
    planner = os.path.join(cpp, 'launch', 'g1_repose_planner.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('artifact', default_value=''),
        DeclareLaunchArgument('artifact_dir', default_value=''),
        DeclareLaunchArgument('checkpoint', default_value=''),
        DeclareLaunchArgument('env', default_value='sim', choices=['sim', 'real']),
        DeclareLaunchArgument(
            'planner_config',
            default_value=[os.path.join(
                cpp, 'config', 'repose_planner', 'g1_'),
                LaunchConfiguration('env'), '_v8_5.yaml']),
        DeclareLaunchArgument('goal_color', default_value='4'),
        DeclareLaunchArgument(
            'camera_ip',
            default_value=os.environ.get('VIBE_CAMERA_ADDRESS', '127.0.0.1')),
        DeclareLaunchArgument('enable_bridge', default_value='true'),
        DeclareLaunchArgument('stand_yaw_teleop', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planner),
            launch_arguments={
                'artifact': LaunchConfiguration('artifact'),
                'artifact_dir': LaunchConfiguration('artifact_dir'),
                'checkpoint': LaunchConfiguration('checkpoint'),
                'env': LaunchConfiguration('env'),
                'planner_config': LaunchConfiguration('planner_config'),
                'goal_color': LaunchConfiguration('goal_color'),
                'camera_ip': LaunchConfiguration('camera_ip'),
                'enable_bridge': LaunchConfiguration('enable_bridge'),
                'stand_yaw_teleop': LaunchConfiguration('stand_yaw_teleop'),
                'approach_only': 'true',
            }.items(),
        ),
    ])
