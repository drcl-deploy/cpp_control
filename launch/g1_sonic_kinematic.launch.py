"""Launch the SONIC tracker and GEAR-SONIC kinematic planner together."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('cpp_control')
    tracker_launch = os.path.join(pkg, 'launch', 'g1_sonic_tracker.launch.py')
    planner_launch = os.path.join(
        pkg, 'launch', 'g1_sonic_kinematic_planner.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('tracker_config', default_value=os.path.join(
            pkg, 'config', 'tracker', 'g1_sonic.yaml')),
        DeclareLaunchArgument('tracker_onnx', default_value=os.path.join(
            pkg, 'models', 'tracker', 'sonic', 'g1_sonic_base.onnx')),
        DeclareLaunchArgument('tracker_manifest', default_value=''),
        DeclareLaunchArgument('planner_config', default_value=os.path.join(
            pkg, 'config', 'planner', 'sonic_kinematic.yaml')),
        DeclareLaunchArgument('planner_model', default_value=os.path.join(
            pkg, 'models', 'planner', 'sonic_kinematic',
            'planner_sonic.onnx')),
        DeclareLaunchArgument('input_source', default_value='unitree'),
        DeclareLaunchArgument('lowstate_topic', default_value='/lowstate'),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('reference_topic',
                              default_value='/tracker/reference'),
        DeclareLaunchArgument('controller_status_topic',
                              default_value='/vibe/controller/status'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tracker_launch),
            launch_arguments={
                'config_path': LaunchConfiguration('tracker_config'),
                'onnx_path': LaunchConfiguration('tracker_onnx'),
                'manifest_path': LaunchConfiguration('tracker_manifest'),
                'motion_path': '',
                'planner': 'true',
                'status_topic': LaunchConfiguration('controller_status_topic'),
                'reference_topic': LaunchConfiguration('reference_topic'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planner_launch),
            launch_arguments={
                'planner_config': LaunchConfiguration('planner_config'),
                'planner_model': LaunchConfiguration('planner_model'),
                'input_source': LaunchConfiguration('input_source'),
                'reference_topic': LaunchConfiguration('reference_topic'),
                'controller_status_topic': LaunchConfiguration(
                    'controller_status_topic'),
                'lowstate_topic': LaunchConfiguration('lowstate_topic'),
                'joy_topic': LaunchConfiguration('joy_topic'),
            }.items(),
        ),
    ])
