"""Bring up the G1 onboard base-state estimator.

    ros2 launch legged_odom g1_odom.launch.py

One node, no controller_manager, no hardware interface. It listens to
`/lowstate` and publishes `/odom`; it never writes a command.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("legged_odom"), "config", LaunchConfiguration("robot_config")]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_config", default_value="g1.yaml"),
            DeclareLaunchArgument("lowstate_topic", default_value="/lowstate"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="false",
                description="publish odom -> pelvis on /tf as well (for rviz)",
            ),
            Node(
                package="legged_odom",
                executable="legged_odom_node",
                name="legged_odom",
                output="screen",
                emulate_tty=True,
                parameters=[
                    config,
                    {
                        "lowstate_topic": LaunchConfiguration("lowstate_topic"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "publish_tf": LaunchConfiguration("publish_tf"),
                    },
                ],
            ),
        ]
    )
