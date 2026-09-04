r"""Closed-loop Repose: the shared Vibe runtime, plus the planner that drives it.

    ros2 launch cpp_control g1_repose_planner.launch.py \\
        artifact:=<run>/<export> [env:=sim|real]
    ros2 run cpp_control repose_console.py        # pick the target colour, live

This is a task wrapper like g1_vibe_repose.launch.py, not a layer on top of it:
both include g1_vibe_sonic.launch.py, so the bridge, the encoder and the
controller come up identically and only `planner:=true` plus this extra node
differ. Open-loop Repose stays exactly what it was.

Prereq beyond the open-loop rig: the camera wire must carry DEPTH —
`camera_streamer.py --depth` on the Orin, or `camera_depth: 1` in the sim
config. The planner refuses to plan without it and says so.

Operator gate: RB enters and locks the nominal stand with the planner idle.
A arms the closed-loop rollout; RB disarms it and returns to stand at any time.
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'artifact', default_value='',
            description='Repose export under $VIBE_ASSET_ROOT/models: <run>[/<export>]'),
        DeclareLaunchArgument(
            'artifact_dir', default_value='',
            description='Repose export directory; relative resolves under $VIBE_ASSET_ROOT'),
        DeclareLaunchArgument('checkpoint', default_value='',
                              description='Artifact step if the directory has multiple exports'),
        # sim by default: the shipping palette was measured off the sim renderer,
        # and a hardware palette is only ever right for the lighting it was
        # recorded under. `env:=real` picks the tuned file instead.
        DeclareLaunchArgument('env', default_value='sim',
                              choices=['sim', 'real'],
                              description='which observe block to load'),
        DeclareLaunchArgument(
            'planner_config',
            default_value=[os.path.join(pkg, 'config', 'repose_planner', 'g1_'),
                           LaunchConfiguration('env'), '.yaml']),
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
        DeclareLaunchArgument(
            'approach_only', default_value='false',
            description='v8.5 smoke gate: allow approach/turn/settle but suppress clips'),
        DeclareLaunchArgument(
            'stand_yaw_teleop', default_value='true',
            description='right-stick heading control while RB stand is active'),
        DeclareLaunchArgument(
            'stand_yaw_rate_deg_s', default_value='45.0',
            description='full-stick manual turn rate while RB stand is active'),
        DeclareLaunchArgument('stand_yaw_accel_deg_s2', default_value='180.0'),
        DeclareLaunchArgument('stand_yaw_deadband', default_value='0.15'),
        DeclareLaunchArgument('stand_yaw_timeout_s', default_value='0.25'),
        DeclareLaunchArgument('stand_yaw_settle_rate_deg_s', default_value='2.0'),
        DeclareLaunchArgument('stand_yaw_settle_gyro_deg_s', default_value='5.0'),
        DeclareLaunchArgument('stand_yaw_settle_error_deg', default_value='5.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(common),
            launch_arguments={
                'artifact': LaunchConfiguration('artifact'),
                'artifact_dir': LaunchConfiguration('artifact_dir'),
                'checkpoint': LaunchConfiguration('checkpoint'),
                'expected_task_family': 'repose',
                'goal_color': LaunchConfiguration('goal_color'),
                'camera_ip': LaunchConfiguration('camera_ip'),
                'enable_bridge': LaunchConfiguration('enable_bridge'),
                'planner': 'true',
                'stand_yaw_teleop': LaunchConfiguration('stand_yaw_teleop'),
                'stand_yaw_rate_deg_s': LaunchConfiguration('stand_yaw_rate_deg_s'),
                'stand_yaw_accel_deg_s2': LaunchConfiguration('stand_yaw_accel_deg_s2'),
                'stand_yaw_deadband': LaunchConfiguration('stand_yaw_deadband'),
                'stand_yaw_timeout_s': LaunchConfiguration('stand_yaw_timeout_s'),
                'stand_yaw_settle_rate_deg_s': LaunchConfiguration(
                    'stand_yaw_settle_rate_deg_s'),
                'stand_yaw_settle_gyro_deg_s': LaunchConfiguration(
                    'stand_yaw_settle_gyro_deg_s'),
                'stand_yaw_settle_error_deg': LaunchConfiguration(
                    'stand_yaw_settle_error_deg'),
            }.items(),
        ),
        Node(
            package='cpp_control',
            executable='g1_repose_planner_node',
            name='g1_repose_planner_node',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('planner_config'),
                'camera_host': LaunchConfiguration('camera_ip'),
                # The same argument the controller gets: one colour, both halves, from
                # boot rather than from the first console keypress.
                'target_color': LaunchConfiguration('goal_color'),
                'approach_only': LaunchConfiguration('approach_only'),
            }],
            # A planner without its controller is a robot holding its last
            # reference. Tear the run down together, like its siblings.
            on_exit=Shutdown(reason='repose planner exited'),
        ),
    ])
