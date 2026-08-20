# Vibe experiment workflow

Each Vibe task launch owns the complete experiment runtime: the CDR/TCP
telemetry server, `encoder_node`, and one `g1_vibe_sonic_node`. Do not start a
second encoder, bridge server, or controller. `Ctrl-C` stops all three.

## 1. Select the experiment environment

Source one setup file in each fresh terminal. Use the real setup on both the
onboard and offboard computers during a real-robot experiment; use the local
setup in every simulation terminal:

```bash
# Real robot (onboard and offboard)
source ~/unitree_ros2/setup.sh

# OR: simulation
source ~/unitree_ros2/setup_local.sh
```

The setup selects everything that differs between deployments:

| Environment | ROS domain | Camera TCP source | Bridge TCP endpoint |
| --- | ---: | --- | --- |
| Real | `0` | `127.0.0.1:5555` | `gilfoyle-rth:5556` |
| Simulation | `1` | `127.0.0.1:5555` | `127.0.0.1:5556` |

The offboard bridge client internally uses isolated ROS domain `71`; no manual
domain export is needed.

## 2. Start the camera source

For a real experiment, run the camera streamer onboard:

```bash
source ~/unitree_ros2/setup.sh
python3 ~/unitree_ros2/cyclonedds_ws/src/vision_encoders/scripts/camera_streamer.py
```

For simulation, start MuJoCo's camera TCP source instead:

```bash
source ~/unitree_ros2/setup_local.sh
cd ~/unitree_ros2/unitree_mujoco/simulate/build
./unitree_mujoco
```

## 3. Launch exactly one experiment

These commands are identical for real and simulation after sourcing the
matching setup. The launch starts the bridge server and encoder, then starts
the controller when the encoder process is running.

```bash
# Repose
ros2 launch cpp_control g1_vibe_repose.launch.py artifact:=g1_repose_big_cube_floor/011pgzbh

# UOLM
ros2 launch cpp_control g1_vibe_uolm.launch.py artifact:=vibe_uolm/xhej6sbd

# PerLoco Grail
ros2 launch cpp_control g1_vibe_perloco_grail.launch.py artifact:=vibe_perloco_grail/0uetimde

# PerLoco OmRe
ros2 launch cpp_control g1_vibe_perloco_omre.launch.py artifact:=vibe_perloco_omre/bc7sz47j

# Dodge
ros2 launch cpp_control g1_vibe_dodge.launch.py artifact:=vibe_dodge/dkwwigny
```

`artifact:=<run>/<export>` names a directory under `$VIBE_ASSET_ROOT/models`
(setup.sh: `<repo>/vibe/models`), so these lines are identical on the desktop
and on the Orin — see [assets.md](assets.md). Drop the `/<export>` when a run
has only one; `artifact_dir:=<path>` still takes a directory, relative or
absolute.

Each launch discovers the matching `.onnx` and `.manifest.json` pair inside the
export directory; checkpoint-specific filenames are not encoded in the launch
files. Most contain one pair and need no other argument. If one contains
several, launch fails instead of guessing; add `checkpoint:=<training_step>`.

Closed-loop Repose runs the same way through `g1_repose_planner.launch.py`, which
adds the planner process to this same runtime — see
[docs/planners/repose/experiments.md](../planners/repose/experiments.md) for the two things that
differ (depth on the camera wire, and the console terminal).

To collect labeled RGB-D snapshots while the robot remains locked in nominal
stand, use the separate sim2sim or hardware workflow in
[Repose color calibration](../planners/repose/color_calibration.md). It has its own
`repose_stream_observe.sh`/`repose_replay_observe.sh` pair and bag directory.

For Repose, UOLM, or PerLoco, stage the motion after the task is running:

```bash
publish-motion /path/to/motion.npz
```

The bridge server is intentionally enabled by default. It uses a listener,
five latest-only subscriptions, and local serialized-message copies; there is
no TCP traffic without a client, but its idle cost is not zero. For a strict
controller/encoder benchmark with no viewing or recording, append
`enable_bridge:=false` to the task launch.

## 4. View or record offboard

In an offboard terminal, source the same experiment environment and run the
same command for both real and simulation:

```bash
# Live encoder-frame + attention viewer
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/vibe/stream_vibes.sh

# The same viewer plus a timestamped bag
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/vibe/stream_vibes.sh record
```

Bags are written under `~/unitree_ros2/bags/` with names such as
`15Aug2026_16_34`. They contain `/enc/frame`, `/enc/tokens`, the attention
mask, `/lowstate`, and `/lowcmd` — plus the planner's three state topics, which are
silent on an open-loop run and need no profile or flag to enable.

Replay the newest completed bag from another fresh terminal:

```bash
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/vibe/replay_vibes.sh
```

Or select one explicitly:

```bash
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/vibe/replay_vibes.sh \
  ~/unitree_ros2/bags/15Aug2026_16_34
```

The replay viewer is read-only and does not join a DDS domain. Its one time
slider synchronizes the raw encoder frame, query-selectable attention overlay,
and selectable scalar traces from `/lowstate` and `/lowcmd`. The image and
attention panels show their offset from the selected bag timestamp so dropped
or delayed samples remain visible. A bag recorded under the planner also gets a
planner pane, held at the cursor rather than interpolated; an open-loop bag
simply has none.

Calibration bags are intentionally separate from experiment bags. Record and
review them with:

```bash
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/planners/repose/repose_stream_observe.sh record
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/planners/repose/repose_replay_observe.sh
```

They default to `~/unitree_ros2/bags/repose_observe/`; the complete hardware and
sim2sim procedure is in
[Repose color calibration](../planners/repose/color_calibration.md).

## Controls and shutdown

- Repose, UOLM, and PerLoco: `RB` stand, `L1` prep, `A` run.
- Dodge: `A` or `RB` engages its nominal reactive reference.
- If the controller prints `STALE VISION TOKENS`, press `Y` for damping if the
  experiment requires it.
- Stop the task launch before starting a different experiment.
- Stop `stream_vibes.sh` with `Ctrl-C` or close its viewer; recording closes
  with the bridge client and viewer.
