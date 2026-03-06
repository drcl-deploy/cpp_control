# cpp_control# cpp_control



C++ control package for G1 robot with layered abstraction:C++ control package for G1 robot locomotion with ONNX policy inference.



```## Directory Structure

Level 0: BaseNode         — robot-agnostic FSM, joystick, timer

Level 1: G1Node       — G1-specific HG messages, gamepad, XR hands```

Level 2: G1Locomotion     — SE2 velocity locomotion policycpp_control/

         G1LocoManip      — loco-manipulation policy with hand tracking├── CMakeLists.txt

```├── package.xml

├── README.md

## Directory Structure├── config/

│   └── locomotion_config.yaml  # Controller configuration

```├── include/

cpp_control/│   ├── common/

├── CMakeLists.txt│   │   ├── gamepad.hpp         # Unitree gamepad parser

├── config/│   │   └── motor_crc_hg.h      # Motor CRC calculation

│   └── g1/│   └── cpp_control/

│       ├── locomotion_config.yaml│       ├── config_loader.hpp   # YAML config loader

│       └── locomanip_config.yaml│       └── onnx_policy.hpp     # ONNX inference wrapper

├── include/├── launch/

│   ├── common/│   └── locomotion_onnx_controller.launch.py

│   │   ├── gamepad.hpp├── models/

│   │   ├── motor_crc_hg.h│   └── locomotion.onnx         # Policy model

│   │   └── types.hpp              ← unified RobotState / RobotCommand├── src/

│   └── cpp_control/│   ├── locomotion_onnx_controller.cpp

│       ├── base.hpp           ← Level 0│   ├── config_loader.cpp

│       ├── config_loader.hpp│   ├── onnx_policy.cpp

│       ├── onnx_policy.hpp│   └── motor_crc_hg.cpp

│       ├── robots/└── thirdparty/                 # ONNX Runtime (symlinked)

│       │   └── g1.hpp    ← Level 1```

│       └── tasks/

│           ├── g1_locomotion_node.hpp  ← Level 2## Control Modes

│           └── g1_locomanip_node.hpp   ← Level 2

├── launch/- **B button**: Zeroing (disable motors)

│   ├── g1_locomotion.launch.py- **Y button**: Damping mode

│   └── g1_locomanip.launch.py- **X button**: Nominal pose (stand up smoothly)

├── models/- **UP button**: Policy control (run ONNX locomotion)

│   ├── locomotion.onnx

│   ├── g1_action_rate_32.onnx## Build

│   └── ...

└── src/```bash

    ├── base.cppcd /home/junchao/drcl_deploy

    ├── config_loader.cppcolcon build --packages-select cpp_control

    ├── motor_crc_hg.cpp```

    ├── onnx_policy.cpp

    ├── robots/## Environment Setup

    │   └── g1.cpp

    └── tasks/Before running, source the appropriate setup script based on your target:

        ├── g1_locomotion_node.cpp

        └── g1_locomanip_node.cpp```bash

```# For simulation (uses loopback interface "lo")

source ~/drcl_deploy/setup_sim.sh

## Control Modes

# For real robot (uses network interface "eno1")

| Input             | Mode           |source ~/drcl_deploy/setup_robot.sh

|-------------------|----------------|```

| Joystick X / GP X | Nominal pose (smooth stand-up) |

| Joystick A / GP ↑ | Policy control |These scripts configure:

| Joystick B / GP B | Zeroing (disable motors) |- ROS2 Humble environment

| Joystick Y / GP Y | Damping |- Cyclone DDS with the correct network interface

- Workspace overlay

## Build

## Usage

```bash

cd ~/ros2/drcl_deploy### With Real Robot

colcon build --packages-select cpp_controlclone unitree_ros2 at https://github.com/unitreerobotics/unitree_ros2, build (TODO: move shell script to repo)

``````bash

source ~/drcl_deploy/setup_robot.sh

## Usageros2 launch cpp_control locomotion_onnx_controller.launch.py

```

### Locomotion

```bash### With Unitree MuJoCo Simulation

ros2 launch cpp_control g1_locomotion.launch.py

ros2 launch cpp_control g1_locomotion.launch.py onnx_model_path:=/path/to/model.onnx**Terminal 1: Start MuJoCo simulation**

```clone unitree_mujoco at https://github.com/unitreerobotics/unitree_mujoco

```bash

### Loco-Manipulationcd ~//unitree_mujoco/simulate_python/

```bash# Make sure config.py has DOMAIN_ID = 1 and INTERFACE = "lo"

ros2 launch cpp_control g1_locomanip.launch.pypython3 unitree_mujoco.py

ros2 launch cpp_control g1_locomanip.launch.py config_path:=/path/to/config.yaml```

```

**Terminal 2: Start Controller**

## Adding a New Robot```bash

source ~/drcl_deploy/setup_sim.sh

1. Create `include/cpp_control/robots/<robot>_base.hpp`ros2 launch cpp_control locomotion_onnx_controller.launch.py

2. Create `src/robots/<robot>_base.cpp````

3. Inherit from `BaseNode`, implement `init_robot()`, `publish_command()`, `num_motors()`

4. Add static library target in `CMakeLists.txt`### Custom Model/Config



## Adding a New Task```bash

# With custom model

1. Create `include/cpp_control/tasks/<task>_<robot>_node.hpp`ros2 launch cpp_control locomotion_onnx_controller.launch.py \

2. Create `src/tasks/<task>_<robot>_node.cpp`    onnx_model_path:=/path/to/your/policy.onnx

3. Inherit from the robot's Level 1 node

4. Override `policy_control()` with task-specific observation + inference# With custom config

5. Add executable target in `CMakeLists.txt`ros2 launch cpp_control locomotion_onnx_controller.launch.py \

    config_path:=/path/to/your/config.yaml

## Dependencies```



- `unitree_hg` (Unitree G1 messages)## Dependencies

- `yaml-cpp`- unitree_hg (Unitree G1 messages)

- ONNX Runtime 1.22.0 (in `thirdparty/`)- yaml-cpp

- ONNX Runtime 1.22.0
