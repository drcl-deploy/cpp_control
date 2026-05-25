"""
Launch G1 Loco-Manipulation Controller + Televuer XR bridge.

Wraps g1_locomanip.launch.py and additionally starts the televuer_bridge_node
from the xr_teleop package, which receives wrist poses over ZMQ from the VR PC
and republishes them as /xr/left_controller and /xr/right_controller.

Usage:
    ros2 launch cpp_control g1_locomanip_with_xr.launch.py
    ros2 launch cpp_control g1_locomanip_with_xr.launch.py vr_pc_ip:=192.168.0.244
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
    locomanip_launch = os.path.join(pkg, 'launch', 'g1_locomanip.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('vr_pc_ip', default_value='192.168.0.244',
                              description='IP of the VR PC running vr_pose_publisher.py'),
        DeclareLaunchArgument('vr_pc_port', default_value='5556',
                              description='ZMQ port the VR PC publishes on'),

        # Locomanip controller (and all its existing arguments)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(locomanip_launch),
        ),

        # Televuer ZMQ → ROS bridge
        Node(
            package='xr_teleop',
            executable='televuer_bridge_node',
            name='televuer_bridge_node',
            output='screen',
            parameters=[{
                'vr_pc_ip': LaunchConfiguration('vr_pc_ip'),
                'port': LaunchConfiguration('vr_pc_port'),
            }],
        ),
    ])
