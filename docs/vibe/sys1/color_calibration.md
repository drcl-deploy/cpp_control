# Sys1 color calibration

Collect independent RGB-D snapshots for the six-color palette, `min_value`,
and `min_rel_sat`. This workflow does not tune geometry or kinematics.

## Build

```bash
cd ~/unitree_ros2/cyclonedds_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cdr_tcp_bridge cpp_control
```

## Sim2sim: three terminals

The simulator config must have `use_joystick: 1` and `camera_depth: 1`.

Terminal 1 — simulator:

```bash
cd ~/unitree_ros2/unitree_mujoco/simulate/build
./unitree_mujoco
```

Terminal 2 — locked nominal stand and observation stream:

```bash
source ~/unitree_ros2/setup_local.sh
ros2 launch cpp_control g1_sys1_observe_calibration.launch.py \
  artifact_dir:=/home/loki/drcl_projects/vibe/logs/rsl_rl/g1_repose_adapt_sonic/wandb_checkpoints/011pgzbh \
  checkpoint:=58000
```

Terminal 3 — offboard viewer and snapshot bag:

```bash
source ~/unitree_ros2/setup_local.sh
ros2 run cpp_control stream_sys1_observe.sh record \
  --samples 30 --negative-samples 60
```

For hardware, run Terminal 2 with `setup.sh`; run Terminal 3 on the offboard
machine with its normal bridge environment.

## Gamepad and viewer

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

## Positive collection: 30 clicks per color

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

## Negative collection: 60 clicks

Use no visible cube:

- 20 empty-workspace frames across normal, bright, and shaded lighting.
- 20 operator/hand/skin frames at varied distances and image positions.
- 20 colored-clothing or colored-object distractor frames.

Negatives tune rejection behavior for the visual gates. A hand covering a cube
is an occlusion test, not a positive color sample.

## Output and validation

Bags are written below `${SYS1_OBSERVE_BAG_DIR}` or
`./sys1_observe_bags`.

```bash
ros2 run cpp_control validate_sys1_observe_bag.py /path/to/bag
```

A complete run contains 30 snapshots for each of six colors and 60 negative
snapshots: 240 selected RGB-D frames total.
