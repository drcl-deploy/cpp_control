"""
Launch file for G1 Locomotion ONNX Controller

Usage:
    ros2 launch cpp_control locomotion_onnx_controller.launch.py
    
    # With custom ONNX model:
    ros2 launch cpp_control locomotion_onnx_controller.launch.py \
        onnx_model_path:=/path/to/your/policy.onnx
"""

import os
import platform
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Determine ONNX Runtime path based on architecture
if platform.machine() in ['x86_64', 'amd64']:
    ONNX_RUNTIME_LIB = "/home/junchao/unitree_deploy_cpp/thirdparty/onnxruntime-linux-x64-1.22.0/lib"
else:
    ONNX_RUNTIME_LIB = "/home/junchao/unitree_deploy_cpp/thirdparty/onnxruntime-linux-aarch64-1.22.0/lib"

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('cpp_control')
    
    # Default paths
    default_config_path = os.path.join(pkg_share, 'config', 'locomotion_config.yaml')
    default_onnx_path = os.path.join(pkg_share, 'models', 'locomotion.onnx')
    
    # Declare launch arguments
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='Path to the locomotion configuration YAML file'
    )
    
    onnx_model_path_arg = DeclareLaunchArgument(
        'onnx_model_path',
        default_value=default_onnx_path,
        description='Path to the ONNX policy model'
    )
    
    # Locomotion controller node
    # Get current LD_LIBRARY_PATH and prepend our ONNX Runtime path
    current_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    new_ld_path = f"{ONNX_RUNTIME_LIB}:{current_ld_path}" if current_ld_path else ONNX_RUNTIME_LIB
    
    locomotion_controller_node = Node(
        package='cpp_control',
        executable='locomotion_onnx_controller',
        name='locomotion_onnx_controller',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'onnx_model_path': LaunchConfiguration('onnx_model_path'),
        }],
        additional_env={'LD_LIBRARY_PATH': new_ld_path},
        # Remap topics if needed
        remappings=[
            # ('lowcmd', '/lowcmd'),
            # ('lowstate', '/lowstate'),
            # ('joy', '/joy'),
        ]
    )
    
    return LaunchDescription([
        config_path_arg,
        onnx_model_path_arg,
        locomotion_controller_node,
    ])
