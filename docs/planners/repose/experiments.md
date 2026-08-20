# The planner experiment workflow

A closed-loop Repose run is a Vibe run with one extra process and one extra
terminal. Everything in [docs/vibe/experiments.md](../../vibe/experiments.md)
applies unchanged — same setup files, same camera source, same viewers, same
bags. This page is only the delta. Algorithm and design:
[planner.md](planner.md). For observation-only RGB-D collection with The controller
locked in nominal stand, use
[color_calibration.md](color_calibration.md), which has complete sim2sim and
hardware workflows.

## 1. What is different

| | open-loop Repose | the planner |
|---|---|---|
| launch | `g1_vibe_repose.launch.py` | `g1_repose_planner.launch.py` |
| processes | bridge, encoder, controller | + `g1_repose_planner_node` |
| camera wire | colour | colour **and depth** |
| who commits references | you, `publish-motion` then `A` | `A` arms the planner; it then commits every ~2 s |
| controls | `RB` stand, `L1` prep, `A` run | `RB` stand + disarm, `A` arm + run — no L1 prep |
| extra terminal | — | the console, to set the target colour and re-arm |
| observe block | — | `env:=sim` or `env:=real` (§2.5) |
| assets it reads | `vibe/models/` | + `vibe/data/` — the clip table and its frames (§3) |

The planner also **observes whenever lowstate is alive**, armed or not. The
console shows the belief before you press `A`, which is the cheapest way to find
out the palette is wrong — no arming, no commit, robot standing still.

## 2. Camera: depth is mandatory

The planner refuses to plan without a depth plane and says so at boot. One extra flag
on the real streamer, one config line in the sim:

```bash
# Real, onboard
python3 ~/unitree_ros2/cyclonedds_ws/src/vision_encoders/scripts/camera_streamer.py --depth

# Simulation: unitree_mujoco/simulate/config.yaml
camera_depth: 1
```

The encoder and the planner both connect to that one server on port `5555`;
it serves multiple clients and the launch hands both halves the same address.

## 2.5 Which observe block: `env:=sim` or `env:=real`

Two configs ship. They differ **only** in the `observe:` block — same clips, same
ladder, same seam, same `version: v7` — so switching them changes what the
planner SEES and nothing about what it does with it.

What the two reads mean, and why the palette alone does not fix either:
[observe_calibration.md](observe_calibration.md).

| | `config/repose_planner/g1_sim.yaml` | `config/repose_planner/g1_real.yaml` |
|---|---|---|
| `env:=` | `sim` (**default**) | `real` |
| `observe.read` | `blobs` — colour first, six candidates, highest wins | `mask` — geometry first, one cube-top plane, modal colour |
| palette | measured off the SIM renderer | measured off `bags/sys1_observe`, 180 labelled hardware frames |
| `chroma_reject` | 0 — every lit pixel votes | 0.06 — a real "none" class |
| `min_rel_sat` | 0.22 | 0.15 (safe only because geometry gates first) |

On the hardware bag, scored by the production `CubeSight`:

| config | colour accuracy | false positives on 60 no-cube frames |
|---|---|---|
| `g1_sim.yaml` | 53.9% | 0/60 |
| `g1_real.yaml` | **83.3%** | 0/60 |

```bash
ros2 launch cpp_control g1_repose_planner.launch.py artifact:=<run>/<export>             # sim
ros2 launch cpp_control g1_repose_planner.launch.py artifact:=<run>/<export> env:=real    # hardware
```

`env` works the same on `g1_repose_planner_calibration.launch.py`. `planner_config:=`
still takes an explicit path and overrides `env` entirely.

`artifact:=<run>/<export>` resolves under `$VIBE_ASSET_ROOT/models` (setup.sh:
`<repo>/vibe/models`), so every line here is identical on the desktop and on the
Orin — see [assets.md](../../vibe/assets.md).

**A/B them without a robot.** Both are scored offline against a labelled bag, so
the comparison costs seconds and needs no hardware:

```bash
ros2 run cpp_control repose_export_observe_bag.py bags/sys1_observe/<run> /tmp/reads
for env in sim real; do
  ros2 run cpp_control repose_planner_selftest \
    --replay /tmp/reads --summary \
    --table /path/to/sys1_clips.npz \
    --config config/repose_planner/g1_${env}.yaml | grep SCORE
done
```

**A/B them live.** `repose_check_observe.sh` (§6) shows the verdict frame by
frame with the controller locked in nominal stand — relaunch with the other
`env:` and watch the same cube. Nothing commits a reference in that rig, so it is
the safe half of a hardware session.

**`sim` is the default on purpose.** `g1_real.yaml`'s palette is bound to the
camera state that recorded its bag — the D435i's auto exposure and auto white
balance were both ON, which is why the deployment's blue face reads cyan. Lock
either, or relight the room, and those twelve numbers are stale: re-record and
re-run the tuner ([color_calibration.md §9](color_calibration.md)).

## 3. Launch

```bash
source ~/unitree_ros2/setup.sh          # or setup_local.sh for simulation

ros2 launch cpp_control g1_repose_planner.launch.py \
  artifact:=g1_repose_big_cube_floor/011pgzbh
```

Same artifact discovery as every other task wrapper, same `enable_bridge`,
same `Ctrl-C` tears down all four processes. `checkpoint:=<step>` is only needed
when the directory holds more than one `.onnx`/`.manifest.json` pair.
`goal_color:=` sets the starting target; the console changes it live.

**The planner needs `vibe/data/` too, not only `vibe/models/`.** Three keys in
the yaml name it, all relative to `$VIBE_ASSET_ROOT`
([assets.md](../../vibe/assets.md)):

| yaml key | file | if it is missing |
|---|---|---|
| `clips` | `data/sys1_clips.npz` | boot throws, naming every root it tried |
| `frames.library` | `data/sys1_library.npz` | same — `source: retargeted` only |
| `frames.path` | `data/retargeted_motions/` | same |

On the Orin, `rsync` those three to the same paths under `<repo>/vibe` and the
yaml needs no edit. Or bake them into one file and point `frames.source: baked`
at it ([planner.md §5](planner.md)) — one npz instead of the dataset.

After launch, press `RB` to enter the nominal SONIC stand. The planner remains idle
and cannot replace that reference. Press `A` once to arm the closed loop. Press
`RB` at any time to disarm The planner immediately and return to nominal stand.

Two lines to read in the boot log, in this order:

```
v7 nominal stand: waist 0.0/0.0/0.0 deg -> camera 45.0 deg down, +0.0 off-axis (mount is 45.0)
The planner v7 ready: 78 clips (F15 B4 L18 R19) | pattern RRF | target blue |
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
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/planners/repose/repose_run_console.sh
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
| `plan` | what the controller is playing. `cost` is the stance residual it must absorb |
| `next` | intent, ~a second before it becomes an act |

## 5. Viewing and recording

Unchanged, and already the planner-aware — `full.yaml` and `light.yaml` carry the
planner's three state topics, so there is no profile to switch:

```bash
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/vibe/stream_vibes.sh
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/vibe/stream_vibes.sh record
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/vibe/replay_vibes.sh
```

A planner bag carries `/vibe/planner/status`, `/vibe/controller/status` and
`/vibe/sonic/goal_color` beside the usual five. The replay viewer grows a
planner pane when they are present — the console's own fields, held at the time
cursor rather than interpolated, because one sample is one decision.

## 6. Observation calibration

Calibration is not a closed-loop The planner run. It launches the same Repose policy
with The controller locked in nominal stand, starts only the observation node, and stores
one labeled RGB-D snapshot per `LT` press. Use its separate pair:

```bash
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/planners/repose/repose_stream_observe.sh record
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/planners/repose/repose_replay_observe.sh
```

Do not use `stream_vibes.sh` for this dataset: it records continuous experiment
telemetry, not the selected calibration frames. Both deployment workflows and
the 30-per-color/60-negative protocol are in
[color_calibration.md](color_calibration.md).

To just LOOK at what the observe block decides — no staging, no clicks, no bag —
use the check rig instead. Same onboard launch, different offboard half:

```bash
bash ~/unitree_ros2/cyclonedds_ws/src/cpp_control/scripts/planners/repose/repose_check_observe.sh
```

It shows the frame, the classifier's label plane, the verdict, and every
candidate the gates threw away. Fitting a palette from a bag and scoring a
config are [color_calibration.md §9](color_calibration.md).

## 7. Reading a run

| symptom | look at | likely |
|---|---|---|
| never leaves `settle`, `belief` says `blind` | `gate` row | **`omega_still` too tight.** `read 0% of frames` means no frame ever passed the quiescence gate, so the planner stands still gathering nothing. Raise it toward the `ω` the row reports while settled |
| never leaves `settle`, `belief` says `3/12` but never carries | `belief` reason column | no read: palette, or depth missing |
| `scan` forever | `belief` reason column | `side_face` — check the boot gaze line |
| the belief names the **wrong colour**, confidently, every frame | `repose_check_observe.sh` candidates row | the palette, not the belief. A systematic misread agrees with itself, so `belief.min_votes` cannot filter it — raising it only makes the planner slower to be wrong. Try `env:=real`; if it is already `real`, the lighting has moved and the palette needs re-fitting |
| one face shows up as **two** candidates | same | it straddles two chromaticity cells. Under `read: blobs` the fragments compete on height and the wrong one can win — this is what `read: mask` exists for |
| commits but the cube leaves view | `plan cost`, `frames` | wrong pool; confirm `version: v7` |
| planner parked | the controller row `idle` | the human owns the robot; press `A` |
| camera dies mid-run | robot returns to the nominal stance and holds | by design — a starved belief plans the stance. `Ctrl-C` when done |
| `STALE VISION TOKENS` | encoder terminal | camera stream stalled; `Y` damps |

`accept_rate` under ~40% during a SETTLE means the gate is too tight; the
planner will settle repeatedly rather than wander, which is safe but goes
nowhere.

Bring-up order and the gates for each phase are in
[planner.md §8](planner.md).
