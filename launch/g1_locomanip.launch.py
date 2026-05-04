"""
Launch G1 Loco-Manipulation Controller.

Launches the G1 locomanip node with both locomotion and locomanipulation
policies. The node subscribes to XR controller topics for end-effector
pose commands and switches between locomotion/locomanip modes via joystick.

Usage:
    ros2 launch cpp_control g1_locomanip.launch.py

    # With custom models:
    ros2 launch cpp_control g1_locomanip.launch.py \
        onnx_model_path:=/path/to/locomotion.onnx \
        locomanip_onnx_path:=/path/to/locomanip.onnx
"""

import os
import platform
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

# ONNX Runtime library path based on architecture
_SRC_DIR = os.path.join(os.path.expanduser('~'), 'drcl_deploy', 'src', 'cpp_control', 'thirdparty')
if platform.machine() in ['x86_64', 'amd64']:
    ONNX_RUNTIME_LIB = os.path.join(_SRC_DIR, 'onnxruntime-linux-x64-1.22.0', 'lib')
else:
    ONNX_RUNTIME_LIB = os.path.join(_SRC_DIR, 'onnxruntime-linux-aarch64-1.22.0', 'lib')


def generate_launch_description():
    pkg = get_package_share_directory('cpp_control')

    # ── Locomotion config ─────────────────────────────────────────
    default_loco_config = os.path.join(pkg, 'config', 'locomotion', 'g1.yaml')

    with open(default_loco_config, 'r') as f:
        loco_cfg = yaml.safe_load(f)
        default_loco_model = loco_cfg.get('onnx_path')
        if default_loco_model:
            default_loco_model = os.path.join(pkg, 'models', default_loco_model)
        else:
            raise ValueError("ONNX model path not found in locomotion config")

    # ── Locomanip config ──────────────────────────────────────────
    default_manip_config = os.path.join(pkg, 'config', 'locomanip', 'g1.yaml')

    with open(default_manip_config, 'r') as f:
        manip_cfg = yaml.safe_load(f)
        default_manip_model = manip_cfg.get('onnx_path')
        if default_manip_model:
            default_manip_model = os.path.join(pkg, 'models', default_manip_model)
        else:
            raise ValueError("ONNX model path not found in locomanip config")

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=default_manip_config),
        DeclareLaunchArgument('onnx_model_path', default_value=default_loco_model),
        DeclareLaunchArgument('locomanip_onnx_path', default_value=default_manip_model),
        DeclareLaunchArgument('left_xr_topic', default_value='/xr/left_controller'),
        DeclareLaunchArgument('right_xr_topic', default_value='/xr/right_controller'),

        Node(
            package='cpp_control',
            executable='g1_locomanip_node',
            name='g1_locomanip_node',
            output='screen',
            additional_env={'LD_LIBRARY_PATH': ONNX_RUNTIME_LIB + ':' + os.environ.get('LD_LIBRARY_PATH', '')},
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
                'onnx_model_path': LaunchConfiguration('onnx_model_path'),
                'locomanip_onnx_path': LaunchConfiguration('locomanip_onnx_path'),
                'left_xr_topic': LaunchConfiguration('left_xr_topic'),
                'right_xr_topic': LaunchConfiguration('right_xr_topic'),
            }],
        ),
    ])
