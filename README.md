# cpp_control

C++ control package for G1 robot locomotion with ONNX policy inference.

## Directory Structure

```
cpp_control/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── locomotion_config.yaml  # Controller configuration
├── include/
│   ├── common/
│   │   ├── gamepad.hpp         # Unitree gamepad parser
│   │   └── motor_crc_hg.h      # Motor CRC calculation
│   └── cpp_control/
│       ├── config_loader.hpp   # YAML config loader
│       └── onnx_policy.hpp     # ONNX inference wrapper
├── launch/
│   └── locomotion_onnx_controller.launch.py
├── models/
│   └── locomotion.onnx         # Policy model
├── src/
│   ├── locomotion_onnx_controller.cpp
│   ├── config_loader.cpp
│   ├── onnx_policy.cpp
│   └── motor_crc_hg.cpp
└── thirdparty/                 # ONNX Runtime (symlinked)
```

## Control Modes

- **B button**: Zeroing (disable motors)
- **Y button**: Damping mode
- **X button**: Nominal pose (stand up smoothly)
- **A button**: Policy control (run ONNX locomotion)

## Build

```bash
cd /home/junchao/drcl_deploy
colcon build --packages-select cpp_control
```

## Environment Setup

Before running, source the appropriate setup script based on your target:

```bash
# For simulation (uses loopback interface "lo")
source ~/drcl_deploy/setup_sim.sh

# For real robot (uses network interface "eno1")
source ~/drcl_deploy/setup_robot.sh
```

These scripts configure:
- ROS2 Humble environment
- Cyclone DDS with the correct network interface
- Workspace overlay

## Usage

### With Real Robot

```bash
source ~/drcl_deploy/setup_robot.sh
ros2 launch cpp_control locomotion_onnx_controller.launch.py
```

### With Unitree MuJoCo Simulation

**Terminal 1: Start MuJoCo simulation**
```bash
cd ~/TextOp/TextOpDeploy/src/unitree_mujoco/simulate_python
# Make sure config.py has DOMAIN_ID = 1 and INTERFACE = "lo"
python3 unitree_mujoco.py
```

**Terminal 2: Start Controller**
```bash
source ~/drcl_deploy/setup_sim.sh
ros2 launch cpp_control locomotion_onnx_controller.launch.py
```

### Custom Model/Config

```bash
# With custom model
ros2 launch cpp_control locomotion_onnx_controller.launch.py \
    onnx_model_path:=/path/to/your/policy.onnx

# With custom config
ros2 launch cpp_control locomotion_onnx_controller.launch.py \
    config_path:=/path/to/your/config.yaml
```

## Dependencies
- unitree_hg (Unitree G1 messages)
- yaml-cpp
- ONNX Runtime 1.22.0
