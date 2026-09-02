"""
Launch the G1 difftrack tracker (diffsimrl ADD/AMP tracking policy).

Usage:
    ros2 launch cpp_control g1_difftrack.launch.py motion:=g1_walk
    ros2 launch cpp_control g1_difftrack.launch.py \
        model_dir:=/abs/path/to/models/tracker/difftrack/g1_walk

    # unattended sim2sim: no joystick, one run, then exit
    ros2 launch cpp_control g1_difftrack.launch.py motion:=g1_walk \
        auto_engage:=true play_duration:=20 exit_when_finished:=true

`motion:=` names a directory under models/tracker/difftrack/ — the one the
exporter wrote, holding difftrack_config.json, motion.bin and policy.onnx.
Several policies for one clip are separate directories (`<clip>__<variant>`),
so comparing them is `motion:=g1_walk__armrand` against the same everything
else.

These policies observe the world frame, so the workflow has to supply a base
pose and twist: mj_sim does (`workflow: drcl_deploy` in the config), hardware
needs `mocap_pose_topic:=/your/mocap/pose`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('cpp_control')
    default_config = os.path.join(pkg, 'config', 'tracker', 'g1_difftrack.yaml')
    models_root = os.path.join(pkg, 'models', 'tracker', 'difftrack')

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=default_config),
        DeclareLaunchArgument('motion', default_value='g1_walk',
                              description='directory under models/tracker/difftrack/'),
        DeclareLaunchArgument('model_dir', default_value='',
                              description='absolute model dir; overrides motion:='),

        # Entry
        DeclareLaunchArgument('entry', default_value='pose',
                              description='pose = hold the nominal pose then engage; '
                                          'direct = engage now (robot already on clip frame 0)'),
        DeclareLaunchArgument('entry_ramp', default_value='0.0',
                              description='>0 also ramps onto the clip first frame — '
                                          'measures badly, the clips start mid-stride'),
        DeclareLaunchArgument('lead_in_duration', default_value='0.0'),

        # Reference placement
        DeclareLaunchArgument('anchor_motion_to_robot', default_value='true'),
        DeclareLaunchArgument('anchor_yaw_to_robot', default_value='true'),
        DeclareLaunchArgument('observe_in_reference_frame', default_value='true'),

        # Run length / safety
        DeclareLaunchArgument('play_duration', default_value='-1.0',
                              description='seconds to track; <=0 = whole clip, or forever if it loops'),
        DeclareLaunchArgument('fall_height', default_value='-1.0',
                              description='<0 keeps the export'"'"'s termination height'),

        # World state on hardware
        DeclareLaunchArgument('mocap_pose_topic', default_value=''),
        DeclareLaunchArgument('mocap_lowpass', default_value='0.0'),

        # Unattended runs
        DeclareLaunchArgument('auto_engage', default_value='false'),
        DeclareLaunchArgument('auto_engage_delay', default_value='1.0',
                              description='dwell after the pose hold settles, before tracking'),
        DeclareLaunchArgument('exit_when_finished', default_value='false'),

        # Every parameter carries an explicit value_type: a launch argument is a
        # string, and without one `play_duration:=10` reaches the node as an
        # integer and kills it on declare_parameter's type check.
        Node(
            package='cpp_control',
            executable='g1_difftrack_node',
            name='g1_difftrack_node',
            output='screen',
            parameters=[{
                'config_path': ParameterValue(LaunchConfiguration('config_path'), value_type=str),
                # An explicit model_dir wins; otherwise motion:= names one under
                # the installed models/tracker/difftrack (resolved in the node).
                'model_dir': ParameterValue(LaunchConfiguration('model_dir'), value_type=str),
                'models_root': models_root,
                'motion': ParameterValue(LaunchConfiguration('motion'), value_type=str),
                'entry': ParameterValue(LaunchConfiguration('entry'), value_type=str),
                'entry_ramp': ParameterValue(LaunchConfiguration('entry_ramp'), value_type=float),
                'lead_in_duration': ParameterValue(LaunchConfiguration('lead_in_duration'), value_type=float),
                'anchor_motion_to_robot': ParameterValue(LaunchConfiguration('anchor_motion_to_robot'), value_type=bool),
                'anchor_yaw_to_robot': ParameterValue(LaunchConfiguration('anchor_yaw_to_robot'), value_type=bool),
                'observe_in_reference_frame': ParameterValue(LaunchConfiguration('observe_in_reference_frame'), value_type=bool),
                'play_duration': ParameterValue(LaunchConfiguration('play_duration'), value_type=float),
                'fall_height': ParameterValue(LaunchConfiguration('fall_height'), value_type=float),
                'mocap_pose_topic': ParameterValue(LaunchConfiguration('mocap_pose_topic'), value_type=str),
                'mocap_lowpass': ParameterValue(LaunchConfiguration('mocap_lowpass'), value_type=float),
                'auto_engage': ParameterValue(LaunchConfiguration('auto_engage'), value_type=bool),
                'auto_engage_delay': ParameterValue(LaunchConfiguration('auto_engage_delay'), value_type=float),
                'exit_when_finished': ParameterValue(LaunchConfiguration('exit_when_finished'), value_type=bool),
            }],
        ),
    ])
