# The planner color calibration

Collect independent RGB-D snapshots for the six-color palette, `min_value`,
and `min_rel_sat`. This workflow does not tune geometry or kinematics.

## 1. Build

Build on the simulator/offboard workspace and on the robot workspace after
syncing this package:

```bash
cd ~/unitree_ros2/cyclonedds_ws
source ~/unitree_ros2/setup.sh          # real robot/offboard
# OR: source ~/unitree_ros2/setup_local.sh  # sim2sim
colcon build --symlink-install --packages-select cdr_tcp_bridge cpp_control
```

## 2. Select the environment

| Workflow | Terminals 1–2 | Terminal 3 | Camera source |
|---|---|---|---|
| Sim2sim | simulator machine, `setup_local.sh` | same machine, `setup_local.sh` | MuJoCo |
| Hardware | onboard, `setup.sh` | offboard, `setup.sh` | onboard D435i with `--depth` |

The streamer creates the isolated offboard bridge domain itself. Do not export
a separate ROS domain manually. Commands assume the checkout is
`~/unitree_ros2`; substitute the robot's checkout prefix if it differs.

## 3. Sim2sim: three terminals

The simulator config must have `use_joystick: 1` and `camera_depth: 1`.

Terminal 1 — simulator:

```bash
source ~/unitree_ros2/setup_local.sh
cd ~/unitree_ros2/unitree_mujoco/simulate/build
./unitree_mujoco
```

Terminal 2 — locked nominal stand and observation stream:

```bash
source ~/unitree_ros2/setup_local.sh
ros2 launch cpp_control g1_repose_planner_calibration.launch.py \
  artifact_dir:=/home/loki/drcl_projects/vibe/logs/rsl_rl/g1_repose_adapt_sonic/wandb_checkpoints/011pgzbh \
  checkpoint:=58000
```

Terminal 3 — offboard viewer and snapshot bag:

```bash
source ~/unitree_ros2/setup_local.sh
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/planners/repose/repose_stream_observe.sh record \
  --samples 30 --negative-samples 60
```

## 4. Hardware: three terminals

Terminal 1 — onboard D435i RGB-D source:

```bash
source ~/unitree_ros2/setup.sh
python3 ~/unitree_ros2/cyclonedds_ws/src/vision_encoders/scripts/camera_streamer.py --depth
```

Terminal 2 — onboard locked nominal stand and observation stream:

```bash
source ~/unitree_ros2/setup.sh
ros2 launch cpp_control g1_repose_planner_calibration.launch.py \
  artifact_dir:=/home/unitree/lkrajan/vibe_models/g1_repose_adapt_sonic/wandb_checkpoints/011pgzbh \
  checkpoint:=58000 \
  planner_config:=/home/unitree/lkrajan/vibe_data/g1_repose.yaml
```

Terminal 3 — offboard viewer and snapshot bag:

```bash
source ~/unitree_ros2/setup.sh
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/planners/repose/repose_stream_observe.sh record \
  --samples 30 --negative-samples 60
```

Terminal 3 never connects to the camera directly. The onboard observation node
publishes paired RGB, depth, labels, and results through the calibration bridge.

## 5. Gamepad and viewer

- `RB`: enter and retain the nominal SONIC stand.
- `LT`: save the next complete paired RGB-D frame as one calibration snapshot.
- One LT press produces one selected frame; holding LT does not repeat.
- The viewer shows current-stage and total selected-frame counts.
- After 30 clicks, the viewer advances automatically to the next color.
- After pink, it advances to `NEGATIVE / NO CUBE` for 60 clicks.
- Keyboard fallback: `Space` clicks, `p` cancels a queued click, `r` restarts
  the current stage, and `q`/`Esc` exits.

The RGB-D stream stays live for the viewer. With `record`, the bag stores only
clicked snapshots and their exact labels, not the continuous video stream.

## 6. Positive collection: 30 clicks per color

Order: red, orange, green, yellow, blue, pink.

For each color collect:

| Lighting | Clicks | Required variation |
|---|---:|---|
| Normal | 10 | near/mid/far; left/center/right |
| Bright | 10 | near/mid/far; left/center/right |
| Shaded | 10 | near/mid/far; left/center/right |

For every click:

1. Put the requested face up.
2. Change distance, lateral placement, or cube rotation.
3. Remove hands and clear the camera-to-cube sightline.
4. Wait about 0.5 s for exposure and depth to settle.
5. Press and release LT once; confirm the count increments once.

Avoid 30 nearly identical placements. Do not click through hand occlusion,
motion blur, or a face transition.

## 7. Negative collection: 60 clicks

Use no visible cube:

- 20 empty-workspace frames across normal, bright, and shaded lighting.
- 20 operator/hand/skin frames at varied distances and image positions.
- 20 colored-clothing or colored-object distractor frames.

Negatives tune rejection behavior for the visual gates. A hand covering a cube
is an occlusion test, not a positive color sample.

## 8. Output, validation, and replay

Bags are written below `${REPOSE_OBSERVE_BAG_DIR}` or, by default,
`~/unitree_ros2/bags/repose_observe/`.

```bash
ros2 run cpp_control repose_validate_observe_bag.py /path/to/bag
```

Review the newest completed bag in the read-only RGB-D replayer:

```bash
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/planners/repose/repose_replay_observe.sh
```

Or open a specific bag directory (passing its `.db3`/`.mcap` file also works):

```bash
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/planners/repose/repose_replay_observe.sh \
  /path/to/bag
```

Use `Left`/`Right` for individual frames, `PageUp`/`PageDown` for stages,
and `Space` for play/pause. The label-overlay checkbox toggles production
classification pixels without changing the bag.

A complete run contains 30 snapshots for each of six colors and 60 negative
snapshots: 240 selected RGB-D frames total.
