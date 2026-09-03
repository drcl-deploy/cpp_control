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
pose and twist. In simulation mj_sim does (`workflow: drcl_deploy` in the
config). On hardware pick exactly one mocap source:

    # the lab OptiTrack, straight off the adaptor
    ros2 launch cpp_control g1_difftrack.launch.py motion:=g1_walk \
        optitrack_topic:=/optitrack_adaptor/mocap_frame \
        optitrack_rigid_body_id:=1

    # anything else that can publish a pose
    ros2 launch cpp_control g1_difftrack.launch.py motion:=g1_walk \
        mocap_pose_topic:=/your/mocap/pose
"""

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('cpp_control')
    default_config = os.path.join(pkg, 'config', 'tracker', 'g1_difftrack_unitree.yaml')
    models_root = os.path.join(pkg, 'models', 'tracker', 'difftrack')
    models_dir = os.path.join(pkg, 'models')

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

        # The rest state: what holds the robot up on either side of a clip.
        # These policies have no standing behaviour of their own.
        DeclareLaunchArgument('start_in_stand', default_value='false',
                              description='enter the rest state on the first state '
                                          'message instead of booting limp in ZEROING — '
                                          'the robot stands, A plays the clip, and it '
                                          'comes back here. Ignored under auto_engage'),
        DeclareLaunchArgument('exit_hold', default_value='0.0',
                              description='seconds to keep the policy driving with the '
                                          'reference clock FROZEN when the clip ends. '
                                          'Measured to make the handover WORSE on a '
                                          'walk clip (the policy flails rather than '
                                          'braking) — 0, the default, skips straight '
                                          'to the gain fade'),
        DeclareLaunchArgument('exit_ramp', default_value='0.0',
                              description='seconds to fade the joint gains from the '
                                          "policy's trained ones back to the yaml's hold "
                                          'gains, ramping the joints to the nominal pose '
                                          'as it goes. 0, the default, hands over on the '
                                          'tick the clip ends — which is what a MOVING '
                                          'robot needs. Raise it only for a rest state '
                                          'that is passive and a clip that ends still'),
        DeclareLaunchArgument('stand_onnx_path', default_value='',
                              description='a SONIC stand export to back the rest state '
                                          'with an actively balancing policy, e.g. '
                                          'tracker/sonic/g1_sonic_base.onnx (relative '
                                          'names resolve under the installed models/). '
                                          'Empty = the nominal-pose PD hold'),
        DeclareLaunchArgument('fall_height', default_value='-1.0',
                              description='<0 keeps the export'"'"'s termination height'),

        # World state on hardware. Exactly one source, or none in simulation
        # where mj_sim/drcl_deploy carries the base pose on the state message.
        DeclareLaunchArgument('optitrack_topic', default_value='',
                              description="the lab's OptiTrack adaptor, e.g. "
                                          '/optitrack_adaptor/mocap_frame'),
        DeclareLaunchArgument('optitrack_rigid_body_id', default_value='1',
                              description='which rigid body in the frame is the pelvis'),
        DeclareLaunchArgument('optitrack_offset', default_value='[0.0, 0.0, 0.0]',
                              description='added to the OptiTrack position, metres — '
                                          'marker cluster centroid to pelvis origin'),
        DeclareLaunchArgument('mocap_pose_topic', default_value='',
                              description='any geometry_msgs/PoseStamped source instead'),
        DeclareLaunchArgument('mocap_lowpass', default_value='0.0'),
        DeclareLaunchArgument('mocap_timeout', default_value='0.2',
                              description='seconds without a pose before refusing to run'),

        # Onboard state estimation — the world-pose source for a lab with no
        # motion capture. docker/estimator runs legged_control2's contact-aided
        # Kalman filter over the robot's own IMU and encoders and publishes
        # nav_msgs/Odometry; see docs/trackers/difftrack_state_estimation.md.
        DeclareLaunchArgument('odom_topic', default_value='',
                              description='nav_msgs/Odometry from an onboard estimator, '
                                          'e.g. /odom'),
        DeclareLaunchArgument('odom_twist_frame', default_value='child',
                              description='which frame the publisher puts twist in: '
                                          '"child" (REP-105, the body frame) or "world"'),
        # Overrides the config yaml's own unitree_world_state. Set it to `none`
        # whenever odom_topic is in use under the unitree workflow, or the
        # simulator's ground truth and the estimator both write the base state.
        DeclareLaunchArgument('unitree_world_state', default_value='',
                              description="'' keeps the yaml's value; 'none' or "
                                          "'sportmode_imu' overrides it"),

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
                'start_in_stand': ParameterValue(LaunchConfiguration('start_in_stand'), value_type=bool),
                'exit_hold': ParameterValue(LaunchConfiguration('exit_hold'), value_type=float),
                'exit_ramp': ParameterValue(LaunchConfiguration('exit_ramp'), value_type=float),
                # A relative name is joined against models_dir in the node, the
                # same way motion:= is joined against models_root. Which tree
                # that is, is a launch-time fact — the INSTALLED share/ — and
                # the node has no business guessing at it.
                'models_dir': models_dir,
                'stand_onnx_path': ParameterValue(
                    LaunchConfiguration('stand_onnx_path'), value_type=str),
                'fall_height': ParameterValue(LaunchConfiguration('fall_height'), value_type=float),
                'optitrack_topic': ParameterValue(LaunchConfiguration('optitrack_topic'), value_type=str),
                'optitrack_rigid_body_id': ParameterValue(
                    LaunchConfiguration('optitrack_rigid_body_id'), value_type=int),
                'optitrack_offset': ParameterValue(
                    LaunchConfiguration('optitrack_offset'), value_type=List[float]),
                'mocap_pose_topic': ParameterValue(LaunchConfiguration('mocap_pose_topic'), value_type=str),
                'mocap_lowpass': ParameterValue(LaunchConfiguration('mocap_lowpass'), value_type=float),
                'mocap_timeout': ParameterValue(LaunchConfiguration('mocap_timeout'), value_type=float),
                'odom_topic': ParameterValue(LaunchConfiguration('odom_topic'), value_type=str),
                'odom_twist_frame': ParameterValue(
                    LaunchConfiguration('odom_twist_frame'), value_type=str),
                'unitree_world_state': ParameterValue(
                    LaunchConfiguration('unitree_world_state'), value_type=str),
                'auto_engage': ParameterValue(LaunchConfiguration('auto_engage'), value_type=bool),
                'auto_engage_delay': ParameterValue(LaunchConfiguration('auto_engage_delay'), value_type=float),
                'exit_when_finished': ParameterValue(LaunchConfiguration('exit_when_finished'), value_type=bool),
            }],
        ),
    ])
