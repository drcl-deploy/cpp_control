# sys1 experiment workflow

A closed-loop Repose run is a Vibe run with one extra process and one extra
terminal. Everything in [docs/vibe/experiments.md](../experiments.md)
applies unchanged — same setup files, same camera source, same viewers, same
bags. This page is only the delta. Algorithm and design:
[planner.md](planner.md).

## 1. What is different

| | open-loop Repose | sys1 |
|---|---|---|
| launch | `g1_vibe_repose.launch.py` | `g1_sys1_repose.launch.py` |
| processes | bridge, encoder, controller | + `g1_sys1_repose_node` |
| camera wire | colour | colour **and depth** |
| who commits references | you, `publish-motion` then `A` | `A` arms the planner; it then commits every ~2 s |
| controls | `RB` stand, `L1` prep, `A` run | `RB` stand + disarm, `A` arm + run — no L1 prep |
| extra terminal | — | the console, to set the target colour and re-arm |

## 2. Camera: depth is mandatory

sys1 refuses to plan without a depth plane and says so at boot. One extra flag
on the real streamer, one config line in the sim:

```bash
# Real, onboard
python3 ~/unitree_ros2/cyclonedds_ws/src/vision_encoders/scripts/camera_streamer.py --depth

# Simulation: unitree_mujoco/simulate/config.yaml
camera_depth: 1
```

The encoder and the planner both connect to that one server on port `5555`;
it serves multiple clients and the launch hands both halves the same address.

## 3. Launch

```bash
source ~/unitree_ros2/setup.sh          # or setup_local.sh for simulation

ros2 launch cpp_control g1_sys1_repose.launch.py \
  artifact_dir:=/home/loki/drcl_projects/vibe/logs/rsl_rl/g1_repose_big_cube_floor/wandb_checkpoints/011pgzbh
```

Same artifact discovery as every other task wrapper, same `enable_bridge`,
same `Ctrl-C` tears down all four processes. `checkpoint:=<step>` is only needed
when the directory holds more than one `.onnx`/`.manifest.json` pair.
`goal_color:=` sets the starting target; the console changes it live.

After launch, press `RB` to enter the nominal SONIC stand. Sys1 remains idle
and cannot replace that reference. Press `A` once to arm the closed loop. Press
`RB` at any time to disarm Sys1 immediately and return to nominal stand.

Two lines to read in the boot log, in this order:

```
v7 nominal stand: waist 0.0/0.0/0.0 deg -> camera 45.0 deg down, +0.0 off-axis (mount is 45.0)
sys1 v7 ready: 78 clips (F15 B4 L18 R19) | pattern RRF | target blue |
              belief 3/8 reads under 0.15 rad/s | camera 127.0.0.1:5555 @ 20 Hz
```

The first is the whole v7 result — if it warns that the mount angle was not
recovered, the run will behave like v6 and there is no point continuing. The
second names the evidence rule the planner will decide on: 3 agreeing reads out
of a window of 8, counted only while the camera is turning slower than
0.15 rad/s.

## 4. The console

Its own terminal, on the **experiment** domain — it publishes the target colour
upstream, and the telemetry bridge only flows the other way.

```bash
source ~/unitree_ros2/setup.sh          # the same file the launch used
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/sys1/run_sys1.sh
```

Keys: `0`-`5` or `r/o/g/y/b/p` set the target, **`R` re-arms on the current
colour**, `q` quits. The colour goes out on `/vibe/sonic/goal_color`, which the
planner and the policy's `object_goal_color` one-hot both read, so they cannot
disagree about what "4" means.

`done` LATCHES, so `R` is the trial loop: solve, `R`, solve again. It also
aborts a trial that went wrong — same key, new `episode`.

Five rows, and what each is for:

```
mode    settle  settle    rung 1  d=R  tips 1  stall 0  burned 2   roll 3  ep 2
belief  ▓▓  11/12 reads ok        r 0.72 m   bearing +12°   spin -8°   412 px
gate    ω 0.04 rad/s   read 68% of frames   2.1 ms/read
plan    settle    cost 0.00 m   yaw +0.00 rad   frames 50 (+10 ramp)
next    R#31      cost 0.34 m   (what a finished reference would commit to)
```

| row | read it for |
|---|---|
| `mode` | the ladder. `roll`/`ep` are the trial counters a run gets scored on |
| `belief` | **the vote.** `11/12 reads` and `3/12 reads` are not the same claim |
| `gate` | placing `omega_still`. A held stance reads ~0.05, a 90° sweep ~0.96 |
| `plan` | what sys0 is playing. `cost` is the stance residual it must absorb |
| `next` | intent, ~a second before it becomes an act |

## 5. Viewing and recording

Unchanged, and already sys1-aware — `full.yaml` and `light.yaml` carry the
planner's three state topics, so there is no profile to switch:

```bash
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/vibe/stream_vibes.sh
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/vibe/stream_vibes.sh record
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/vibe/replay_vibes.sh
```

A sys1 bag carries `/vibe/sys1/status`, `/vibe/sys0/status` and
`/vibe/sonic/goal_color` beside the usual five. The replay viewer grows a
planner pane when they are present — the console's own fields, held at the time
cursor rather than interpolated, because one sample is one decision.

## 6. Reading a run

| symptom | look at | likely |
|---|---|---|
| never leaves `settle`, `belief` says `blind` | `gate` row | **`omega_still` too tight.** `read 0% of frames` means no frame ever passed the quiescence gate, so the planner stands still gathering nothing. Raise it toward the `ω` the row reports while settled |
| never leaves `settle`, `belief` says `3/12` but never carries | `belief` reason column | no read: palette, or depth missing |
| `scan` forever | `belief` reason column | `side_face` — check the boot gaze line |
| commits but the cube leaves view | `plan cost`, `frames` | wrong pool; confirm `version: v7` |
| planner parked | sys0 row `idle` | the human owns the robot; press `A` |
| camera dies mid-run | robot returns to the nominal stance and holds | by design — a starved belief plans the stance. `Ctrl-C` when done |
| `STALE VISION TOKENS` | encoder terminal | camera stream stalled; `Y` damps |

`accept_rate` under ~40% during a SETTLE means the gate is too tight; the
planner will settle repeatedly rather than wander, which is safe but goes
nowhere.

Bring-up order and the gates for each phase are in
[planner.md §8](planner.md).
