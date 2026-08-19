"""Launch only the GEAR-SONIC kinematic planner.

Use this after starting g1_sonic_tracker.launch.py with planner:=true when the
planner and tracker should live in separate terminals.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('cpp_control')
    default_config = os.path.join(
        pkg, 'config', 'planner', 'sonic_kinematic.yaml')
    default_model = os.path.join(
        pkg, 'models', 'planner', 'sonic_kinematic', 'planner_sonic.onnx')

    return LaunchDescription([
        DeclareLaunchArgument('planner_config', default_value=default_config),
        DeclareLaunchArgument('planner_model', default_value=default_model),
        DeclareLaunchArgument('input_source', default_value='unitree',
                              description='unitree or joy'),
        DeclareLaunchArgument('reference_topic',
                              default_value='/tracker/reference'),
        DeclareLaunchArgument('controller_status_topic',
                              default_value='/vibe/controller/status'),
        DeclareLaunchArgument('lowstate_topic', default_value='/lowstate'),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),

        Node(
            package='cpp_control',
            executable='g1_sonic_kinematic_planner_node',
            name='g1_sonic_kinematic_planner_node',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('planner_config'), {
                'model_path': LaunchConfiguration('planner_model'),
                'input_source': LaunchConfiguration('input_source'),
                'reference_topic': LaunchConfiguration('reference_topic'),
                'controller_status_topic': LaunchConfiguration(
                    'controller_status_topic'),
                'lowstate_topic': LaunchConfiguration('lowstate_topic'),
                'joy_topic': LaunchConfiguration('joy_topic'),
            }],
        ),
    ])
