# SONIC kinematic planner

This is a small cpp_control-native integration of NVIDIA's published
GEAR-SONIC V2 kinematic planner. It is not a copy of the reference runtime.
The planner owns ONNX generation and operator intent; the existing
`g1_sonic_node` remains the only low-level policy/controller.

```text
Unitree remote or /joy
        |
        v
g1_sonic_kinematic_planner_node
        |  /tracker/reference (MotionReference, 83-column IL wire layout)
        v
g1_sonic_node
        |  /lowcmd
        v
       G1
```

## Model and build

Download the released model into the package's local model location:

```bash
cd cyclonedds_ws/src/cpp_control
scripts/planners/setup_sonic_kinematic.sh
```

Pass `--force` to replace an existing download. The script checks for the
Hugging Face CLI and verifies the final artifact before returning success.

The resulting path must be:

```text
models/planner/sonic_kinematic/planner_sonic.onnx
```

The file is about 739 MiB and is deliberately ignored by Git. The
implementation needs ONNX Runtime and the existing cpp_control dependencies;
it adds no TensorRT, CUDA, or new ROS message contract.

Build and validate:

```bash
source setup.sh                 # setup_local.sh for simulation plumbing
cd cyclonedds_ws
colcon build --symlink-install --packages-select cpp_control
source install/setup.bash

# Algorithm/wire tests. Add the model path to smoke-test real inference.
ros2 run cpp_control sonic_kinematic_selftest
ros2 run cpp_control sonic_kinematic_selftest \
  "$(ros2 pkg prefix cpp_control)/share/cpp_control/models/planner/sonic_kinematic/planner_sonic.onnx"
```

## Run

Tracker and planner together:

```bash
ros2 launch cpp_control g1_sonic_kinematic.launch.py
```

In separate terminals:

```bash
# Terminal 1: the tracker owns robot control. planner:=true is required.
ros2 launch cpp_control g1_sonic_tracker.launch.py \
  motion_path:='' planner:=true

# Terminal 2: the planner owns only high-level references.
ros2 launch cpp_control g1_sonic_kinematic_planner.launch.py
```

For a standard `sensor_msgs/Joy` source, add `input_source:=joy`. The generic
Xbox axis map is in `config/planner/sonic_kinematic.yaml` and can be overridden
there if the driver numbers the axes differently.

## Operator controls

The planner deliberately does not interpret X, Y, A, B, or RB. Those stay with
the low-level state machine and tracker.

| Input | Planner action |
|---|---|
| Left stick Y | Forward/backward motion and speed |
| Right stick X | Integrate desired facing yaw |
| D-pad Left/Right | Browse all 27 modes; no behavior changes yet |
| LT/L2 press | Commit the highlighted mode |

The terminal always shows both `active` and `menu`. A trailing `*` means the
highlighted menu item is only a preview. `LT COMMIT` is logged explicitly when
the active mode changes.

On the tracker, A still enters POLICY and arms fresh planner references; RB
disarms the planner and locks the nominal SONIC stand. The planner will publish
nothing until `ControllerStatus.accepting` is true. If gamepad or robot state
goes stale, it issues one Idle plan and stops advancing commands.

## Modes

All V2 modes are browseable and committable:

| ID | Mode | ID | Mode | ID | Mode |
|---:|---|---:|---|---:|---|
| 0 | Idle | 9 | Idle Boxing | 18 | Stealth Walk |
| 1 | Slow Walk | 10 | Walk Boxing | 19 | Injured Walk |
| 2 | Walk | 11 | Left Jab | 20 | Ledge Walking |
| 3 | Run | 12 | Right Jab | 21 | Object Carrying |
| 4 | Squat | 13 | Random Punches | 22 | Stealth Walk 2 |
| 5 | Kneel Two Legs | 14 | Elbow Crawling | 23 | Happy Dance Walk |
| 6 | Kneel One Leg | 15 | Left Hook | 24 | Zombie Walk |
| 7 | Lying Facedown | 16 | Right Hook | 25 | Gun Walk |
| 8 | Hand Crawling | 17 | Forward Jump | 26 | Scare Walk |

Walk/Run modes automatically request learned Idle while the left stick is
neutral. Crawling, boxing, ground, and style modes retain their committed
identity at zero speed. Forward Jump and strikes are committed behaviors, so
test those only with adequate clearance and an operator ready on RB.

## Implementation notes

- The ONNX ABI is validated at startup: 11 named mixed-dtype inputs and the
  `mujoco_qpos` plus `num_pred_frames` outputs.
- Four 30 Hz MuJoCo-qpos context frames are sampled from the reference the
  tracker reports it is actually playing. The model output is truncated using
  `num_pred_frames`, interpolated to 50 Hz, then cross-faded at handoff.
- Joint state is canonical `g1::Motion` MJ order internally. Publishing uses
  the existing full 83-column `MotionReference` IL-order encoder, including
  derived joint/root velocity and zero object-contact channels.
- The tracker remains the clock authority. Reference swaps preserve tracker
  observation history, and heading alignment stays in the existing
  `MotionClock`; the planner does not create a second robot-control path.
- The default prediction mask permits 9–11 tokens (36–44 model frames), which
  matches the current GEAR-SONIC V2 TensorRT deployment.

Configuration lives in `config/planner/sonic_kinematic.yaml`. The core adapter
is `include/cpp_control/planners/sonic_kinematic.hpp` with its implementation in
`src/planners/sonic_kinematic.cpp`; it is ROS-free and independently testable.

## Upstream resources

- Model: `nvidia/GEAR-SONIC` on Hugging Face, file `planner_sonic.onnx`
- Reference deploy: `GR00T-WholeBodyControl/gear_sonic_deploy/deploy.sh`
- Model contract: `docs/source/references/planner_onnx.md` in the upstream repo
- Original gamepad behavior: `docs/source/tutorials/gamepad.md` in the upstream
  repo

Hardware rollout should begin suspended or in a clear support rig: verify
LowState freshness, `/tracker/reference` QoS, A arm, RB disarm, and disconnect
fallback before a free-standing test.
