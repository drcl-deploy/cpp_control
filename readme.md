# cpp_control

Modular C++ policy deployment for ROS 2: shared control modes, robot backends,
manifest-driven inference and motion playback. Controllers run in simulation
or on hardware; external planners exchange references over ROS topics.

## install

For the released ViBe samples and complete setup, use
[vibe_ws](https://github.com/drcl-deploy/vibe_ws#install).
For another workspace, follow the [Unitree](workflows/unitree.md) or
[DRCL](workflows/drcl_deploy.md) backend setup, provision ONNX Runtime using
`vision_encoders/scripts/setup_onnxruntime.sh`, then build from the sourced ROS workspace:

```bash
colcon build --symlink-install --packages-select cpp_control \
  --cmake-args -DONNXRUNTIME_ROOT=/path/to/onnxruntime-sdk
```

A stable `thirdparty/onnxruntime` link also selects the SDK. See
[ViBe integration](docs/vibe.md) for offboard-only builds and viewing tools.

## usage

```bash
# SONIC: stand, then track a reference
ros2 launch cpp_control g1_sonic_tracker.launch.py \
  onnx_path:=/path/to/policy.onnx motion_path:=/path/to/motion.npz

# ViBe: encoder + tracker + optional telemetry
ros2 launch cpp_control g1_vibe_sonic.launch.py \
  policy_path:=/path/to/policy.onnx encoder_path:=/path/to/encoder.onnx \
  motion_path:=/path/to/motion.npz

# Live attention; add "record" to save a bag
bash cyclonedds_ws/src/cpp_control/scripts/vibe/stream_vibes.sh
bash cyclonedds_ws/src/cpp_control/scripts/vibe/replay_vibes.sh [BAG_DIRECTORY]
```

| Robot / controller | Launch |
|---|---|
| G1 locomotion | `g1_locomotion.launch.py` |
| G1 SONIC / TextOp tracking | `g1_sonic_tracker.launch.py` / `g1_textop_tracker.launch.py` |
| G1 ViBe, any supported export | `g1_vibe_sonic.launch.py` |
| G1 ViBe task wrappers | `g1_vibe_{walk,parkour,repose,uolm,dodge}.launch.py` |
| MiniPi locomotion / dive | `mini_pi_locomotion.launch.py` / `mini_pi_dive.launch.py` |

ViBe motion tasks use **RB** stand → **L1** prep → **A** run; **Y** selects
damping. A completed clip leaves the tracker available for another reference.
Task wrappers check the manifest family; policy paths select the actual export.
The four sample runs and external Repose planner commands live in
[vibe_ws](https://github.com/drcl-deploy/vibe_ws#usage).

## docs

| Guide | Contents |
|---|---|
| [Architecture](docs/ethos.md) | Robot/task layers, backends, control modes and extension points |
| [Policy serving](docs/onnx_policies.md) | Existing ONNX stacks and manifest-driven inference |
| [SONIC tracker](docs/trackers/custom_sonic.md) | Export contract, prep, stand and tracker behavior |
| [Motion](docs/motion.md) | Reference formats, playback and publishing clips |
| [ViBe integration](docs/vibe.md) | Task families, artifacts, planner interface and telemetry |

## acknowledgements

- [BeyondMimic](https://github.com/HybridRobotics/motion_tracking_controller)
- [TextOp](https://github.com/TeleHuman/Textop)
- [SONIC](https://github.com/NVlabs/GR00T-WholeBodyControl)
