# Running difftrack — sim2sim, and the robot

The run-book. [`difftrack.md`](difftrack.md) is the reference: what the export
contains, how the observation is built, why the entry works the way it does.
This is the sequence of commands, in order, for the three ways this task gets
run — one script, by hand, and on hardware.

---

## 0. What actually runs

Three processes, and knowing which is which saves most of the debugging:

| | what it is | who starts it |
|---|---|---|
| **the controller** | `g1_difftrack_node` — reads the state, runs `policy.onnx`, publishes joint commands. **No viewer, no simulator, no robot in it.** | `ros2 launch cpp_control g1_difftrack.launch.py` |
| **the plant** | `unitree_mujoco` in simulation, the robot's own DDS on hardware | `unitree_mujoco`, or the robot |
| **the world pose** | absolute position, heading, world velocity | `unitree_mujoco` via SportModeState + IMU (§2b); on hardware, OptiTrack |
| **the rest state** | what holds the robot up between clips — the tracking policy has no standing behaviour of its own | the same node: `stand_onnx_path` (SONIC stand policy) or the nominal-pose hold (§2c) |

**Where this workspace is.** `cpp_control` is a package in
`unitree_ros2/cyclonedds_ws/src/`, beside `unitree_hg`. It used to live in a
second colcon workspace (`drcl/`) alongside `mj_sim`, `messages` and `assets`;
that workspace is gone and the `drcl_deploy` plant with it. Every number below
marked *(mj_sim)* was taken there and is kept for comparison — it is not
reproducible in this tree without cloning those three packages back in.

Two consequences worth having in mind before the first command:

- **`g1_difftrack.launch.py` on its own draws nothing and does nothing.** It
  loads the policy, prints its `difftrack loaded` block, and waits forever for a
  state message. The MuJoCo window belongs to the plant — `unitree_mujoco`.
- **The controller has to be commanding before the robot is free to fall.**
  `unitree_mujoco` steps physics from the moment it starts, and its DDS bridge
  takes 716 ms to come up — so the robot integrates limp before any controller
  can command it, and the difftrack hold only survives a limp window of ≤0.36 s.
  Start the controller FIRST, wait for `difftrack loaded`, and start the
  simulator with `-c` (`--wait-for-cmd`, from
  [`workflows/patches/`](../../workflows/patches/README.md)), which holds the
  physics at the reset pose until the first `LowCmd`. A G1 folding from 0.79 m
  is this, and it is never a policy result. *(On the retired mj_sim plant the
  same symptom had a different cause: it re-read the publisher count on
  `/robot_command` every iteration and drove `kp=0 kd=0.1` itself when it found
  none. `unitree_mujoco` does not check at all.)*

**World state is not optional.** These policies were trained with
`imitation.global_obs=true` and observe absolute root height, absolute yaw, and
world-frame linear and angular velocity. An IMU supplies none of it. The node
refuses to engage without a world pose rather than feed the policy zeros.

---

## 1. Once: export, build, verify

```bash
DIFFSIMRL=/path/to/diffsimrl
WS=/path/to/unitree_ros2/cyclonedds_ws          # this colcon workspace
PKG=$WS/src/cpp_control

# export a trained policy out of diffsimrl (its env, not this one)
cd $DIFFSIMRL/code && conda activate diffsimrl
python scripts/export_to_drcl_cpp_control.py ../logs/<run_dir> \
    --out $PKG/models/tracker/difftrack --trace-steps 400

# build — the launches read install/, NOT the source tree
cd $WS
bash src/cpp_control/workflows/conda_env/build.sh --packages-select cpp_control

# verify the export against the golden trace: no robot, no sim, no ROS
source src/cpp_control/workflows/conda_env/runenv.sh
./build/cpp_control/difftrack_selftest \
    $(ros2 pkg prefix cpp_control)/share/cpp_control/models/tracker/difftrack/g1_walk
```

A clean self-test looks like this, and a failure here means nothing downstream
is worth running:

```
worst steady state  2.6e-07   (bound 2e-5)
worst loop repeat   2.1e-03   (bound 3e-3, LOOPING clips only)
worst action        3.1e-06   (against an action RMS of 0.71)
PASS
```

**A fresh export that "does nothing" is a skipped rebuild**, every time.

---

## 2. sim2sim on the drcl plant — RETIRED, kept for its numbers

> **This section does not run in this tree.** It describes `mj_sim` over
> `messages/G1State`, which lived in the `drcl/` colcon workspace that
> `cpp_control` no longer sits in. `-W drcl` says so and exits rather than
> failing inside a missing `import mj_sim`. The script's code for it is intact;
> clone `drcl-deploy/{messages,mj_sim,assets}` back in beside this package and
> it works again.
>
> **Go to [§2b](#2b-sim2sim-on-the-unitree-workflow) for the workflow that
> ships.** It is the same policy, the same observation builder and the same
> entry logic over the message path the robot actually speaks, and its numbers
> agree with this one to 5 mm.

```bash
cd $DRCL          # the retired drcl/ workspace root -- not this tree

# watch it: viewer stays up, tracking until Ctrl-C
bash cpp_control/scripts/run_difftrack_sim2sim.sh -W drcl -w -e stand g1_walk

# measure it: one SUMMARY line per run, no operator
bash cpp_control/scripts/run_difftrack_sim2sim.sh -W drcl -d 30 -r 3 g1_walk
```

That is the whole of section 3 in one command. The script sources the
environment, generates a scene with the bring-up weld cleared, writes an mj_sim
config for it, starts the controller **first**, waits for `difftrack loaded`,
starts the simulator, and asserts exactly one publisher on `/robot_state`.

```
motion                      rep  steps     mean_err  max_err   min_h     fell_at
g1_walk                       1  750/750   0.312     0.642     0.661     none
g1_walk                       2  750/750   0.301     0.544     0.638     none
```

A run is clean when `fell_at=none`. `mean_err` is the root tracking error in
metres against the reference the policy was actually given.

| flag | meaning |
|---|---|
| `-w` | watch: one motion, no budget, no summary, viewer up until Ctrl-C |
| `-e rsi` | (default) robot starts standing **on** the clip's first frame — the comparison against diffsimrl's own numbers |
| `-e stand` | robot starts in the model's pose, holds the nominal pose, then picks the clip up — the hardware sequence |
| `-d N` | tracking budget, seconds |
| `-r N` | repeats per motion |
| `-f` | free-run the simulator instead of pacing it to the wall clock — faster sweeps, identical physics, but the viewer plays at 1.4x-1.8x |
| `-R` | record an mp4 per run into `cpp_control/recordings/`, named for the motion — the same recorder F9 drives, started at t=0 |
| `-k` | keep the logs and print where they are |
| *(no motion)* | every export under `models/tracker/difftrack/` |

Measured here, 15 s, `g1_walk`: `-e rsi` 0.30-0.32 m mean, `-e stand`
0.32-0.36 m. Under `-w` with the clip looping, the stand entry settles at
0.24-0.32 m over thousands of steps — the RSI mean is dominated by a start
transient, because MuJoCo cannot carry velocity across a reset and the robot
begins at rest against a clip entering at 0.97 m/s.

### speed — it runs in real time, and that is checked

`run_mj_sim.py` paces the loop against the wall clock, and says so:

```
[rate] physics 500 Hz, paced to 1x real time (2.00 ms of wall clock per step)
[rate] 1.00x real time (500 Hz physics, 50.0 Hz control)
```

Measured over 90 s: **4503 control steps in 90.06 s = 50.00 Hz, 1.000x**.

Upstream `mj_sim` does not do this, and the difference is visible rather than
subtle. Its loop is paced by an rclpy rate at **1000 Hz** while the G1 model's
physics timestep is **2 ms (500 Hz)**, so a wall second buys up to two seconds of
simulation; it does not quite keep up, and what you get is **1.4x-1.8x**, moving
with machine load — measured at 70-88 Hz control against a 50 Hz policy across
fourteen runs before this was fixed.

**Nothing about the physics was ever wrong**, which is why the older tracking
numbers still stand: `state_decimation: 10` ticks the control loop off the state
stream, so the policy gets exactly 10 x 2 ms = the 20 ms of physics per control
step it trained with, whatever the wall clock is doing. What was wrong was the
playback, and it made the gait look like something it is not — `g1_walk` is a
1.033 s cycle (`wrap_cycles: 3` over a 3.1 s table), so **1.94 steps/s at 1x**, a
normal brisk walk. Shown at 1.75x that is 3.4 steps/s: a jog.

`--speed 0.5` is worth knowing for looking at a single footfall, and `-f` on the
script (`--speed 0`) free-runs for sweeps.

### the viewer

`scripts/run_mj_sim.py` is mj_sim with a camera that stays on the robot —
azimuth, elevation and distance are yours to drag, and only the look-at point
follows the pelvis, the way `crl-humanoid-ros`'s monitor does it. Upstream's
free camera never moves, and a G1 at 1 m/s leaves the frame in about four
seconds.

| key | |
|---|---|
| `space` | pause / resume |
| `V` | camera follow on/off |
| `F9` | start / stop recording an mp4 (see below) |
| `Y` | put the robot on the ground (clears the bring-up weld) and pause |
| `E` | body frames |
| `[` `]` | hanging height, while the weld is active |

Tune it with a `camera:` block in the mj_sim yaml — `follow`, `body`,
`azimuth`, `elevation`, `distance`, `height_offset`, `smooth` (seconds of lag on
the look-at point; the pelvis bobs several centimetres per step and feeding that
straight to the camera shakes the whole scene). Defaults are in the script's
docstring.

### recording

`F9` starts and stops an mp4 in `cpp_control/recordings/`
(`<prefix>_<timestamp>.mp4`); `-R` on the sim2sim script, or `--record` on the
simulator, starts one at t=0 instead. It needs `ffmpeg` on PATH and nothing
else.

The scene is rendered a SECOND time into MuJoCo's offscreen framebuffer and
piped into ffmpeg — the same approach as `crl-humanoid-ros`'s monitor, and the
reason the video is 720p whatever the window size, is unaffected by anything
sitting on top of the window, and carries no UI overlay. Encoding runs on its
own thread; a 720p30 recording held 1.00x real time over a 40 s run.

Frames are clocked on **simulation** time by default, so the file plays back at
1x even under `-f`, and a pause is cut out rather than recorded as a freeze.
`clock: wall` gives you the opposite: exactly what you watched, at the speed you
watched it.

Settings live in a `recording:` block in the mj_sim yaml (`dir`, `size`, `fps`,
`clock`, `encoder`, `crf`, `prefix`) and every one of them has a
`DRCL_RECORD_*` environment override — `CRL_RECORD_*` is read too, so a shell
set up for the crl-humanoid-ros monitor works here unchanged. Defaults and the
measured render costs are in `scripts/frame_recorder.py`'s docstring.

The file is finalised on Ctrl-C, on closing the window, and even when the
simulator is `kill -9`'d (ffmpeg outlives it by a moment and closes the stream
itself) — what it does not survive is killing ffmpeg.

---

## 2b. sim2sim on the UNITREE workflow

**This is the workflow, and it is the default.** The same policy, the same
observation builder and the same entry logic, through `unitree_hg`
`LowState`/`LowCmd` on CycloneDDS against
[`unitree_mujoco`](../../workflows/unitree.md) — the transport, the IDL, the
motor order, the CRC and the `mode_machine` the real G1 speaks.

```bash
cd $WS

# watch it
bash src/cpp_control/scripts/run_difftrack_sim2sim.sh -w -e stand g1_walk

# measure it
bash src/cpp_control/scripts/run_difftrack_sim2sim.sh -d 30 -r 3 g1_walk
```

Measured in this tree after the move, `g1_walk`, 15 s, `-e rsi`: `750/750`,
`mean_err` 0.322-0.325 m, `fell_at=none` — the same numbers the section below
records, which is the check that the move changed nothing.

The script switches the middleware itself (`DRCL_WORKFLOW=unitree source
runenv.sh`), generates a unitree scene, starts the controller **first** with
`config/tracker/g1_difftrack_unitree.yaml`, waits for `difftrack loaded`, starts
`unitree_mujoco` with `--wait-for-cmd`, and asserts exactly one publisher on
`/lowstate` and one on `/lowcmd`.

Measured, 15 s, both entries, every shipped export — and against the drcl
reference in the same conditions:

```
                              unitree_mujoco            mj_sim (reference)
motion                  rsi     rsi -l 0.5   stand      rsi
g1_walk                 0.320   0.242        0.271      0.325
g1_walk__armrand        0.132   0.141        0.138      (falls at step 80 w/ -l)
g1_walk__delay          0.262   0.345        0.296      0.180
```

All `750/750` steps, `fell_at=none`, `min_height` 0.72-0.74 m. The headline is
that `g1_walk` comes out at **0.320 m on unitree_mujoco against 0.325 m on
mj_sim** — two different models, two different integrators, two different
message paths, agreeing to 5 mm. That agreement is the point of having both.

### two things this workflow found

Neither was visible under mj_sim, and both matter for hardware.

**1. `unitree_mujoco` runs physics on a limp robot at startup.** It steps for as
long as its DDS bridge takes to come up — 716 ms of simulated time, measured —
before any controller can command anything. The difftrack hold survives a limp
window of ≤0.36 s and fails at ≥0.38 s, so every run on the stock simulator was
measuring a startup race. See
[`workflows/patches/`](../../workflows/patches/README.md): three small changes,
the last of which (`--wait-for-cmd`, opt-in) takes the window to zero.

**2. The hold gains' damping is unstable when the PD is an external torque.**
`kd = kp/20` — what `g1_difftrack.yaml` and `g1_difftrack_hw.yaml` carry — pins
twelve leg joints at *full* torque with the sign flipping between samples, while
the position errors are still ~0.06 rad. That is an oscillating loop, not a
balance failure, and it puts the robot on the floor in a second.

It never appeared on mj_sim because mj_sim rewrites MuJoCo's actuator
`gainprm`/`biasprm`: the PD is a **position servo inside the solver**, whose
damping MuJoCo integrates implicitly. `unitree_mujoco` computes
`tau = kp(q*-q) + kd(dq*-dq)` on its own 1 kHz thread and writes it to `ctrl` —
**explicit** damping, and not even synchronous with the physics, since the
simulator steps in bursts to keep up with the wall clock.

**The robot's motor controller is the second kind, not the first.** Measured, on
the same ramp and the same plant:

| kd | outcome |
|---|---|
| kp/20 | fell — 12 joints at full torque, sign alternating |
| kp/30 | held, zero saturation |
| **kp/40** | held, zero saturation, tracking within 0.03 rad — what the unitree config ships |
| kp/100 | held, indistinguishable from kp/40 |

If you take the hold gains to hardware, take *this* kd, not mj_sim's.

### the three configs

| | plant | transport | world pose |
|---|---|---|---|
| `g1_difftrack.yaml` | mj_sim | `messages/G1State` + `G1Command` | ground truth, on the state message |
| `g1_difftrack_unitree.yaml` | unitree_mujoco | **`unitree_hg` LowState/LowCmd** | `unitree_world_state: sportmode_imu` |
| `g1_difftrack_hw.yaml` | the G1 | **`unitree_hg` LowState/LowCmd** | OptiTrack |

The unitree sim config differs from the hardware config in **one line** — the
world-state source — and that line is the one thing simulation can do and
hardware cannot.

### the world pose, which is the whole difficulty

These policies observe absolute root height, absolute yaw and world-frame linear
and angular velocity (`imitation.global_obs=true`). `LowState` has none of it: an
IMU has no position and no heading.

`unitree_world_state: "sportmode_imu"` assembles it from what this workflow does
publish, and it is named for what it **reads** rather than what it means:

| | source | under `unitree_mujoco` | on the robot |
|---|---|---|---|
| position | `SportModeState.position` | MuJoCo's `frame_pos` sensor on the pelvis site | not published with the motion service off; legged odometry when it is, and it drifts |
| world linear velocity | `SportModeState.velocity` | `frame_vel` | same |
| orientation | `LowState.imu_state.quaternion` | `imu_quat` | real IMU; **yaw drifts** |
| world angular velocity | `R(quat) · LowState.imu_state.gyroscope` | `imu_gyro`, rotated | real IMU |

Under `unitree_mujoco` all four are exact. The `imu` site in `g1_29dof.xml` sits
at `pos="0 0 0"` in the pelvis body, so `frame_pos`/`frame_vel`/`imu_quat` are
that body's world pose and twist, not something offset from it — the same state
mj_sim puts in `G1State.base_pose` / `base_twist`.

Two things are load-bearing in that table:

- **The gyro is rotated.** Every gyroscope reports body-local angular velocity;
  the observation wants world. Copying it raw is wrong in exactly the way that
  is invisible while the robot stands upright — it is the bug that cost
  diffsimrl's own sim2mujoco path a 4x regression in tracking duration.
- **The setting is off by default and warns loudly when on.** The node refuses
  to engage without a world pose (`difftrack cannot engage: no world base
  state`) rather than feeding the policy a plausible-looking identity, and
  `g1_difftrack_hw.yaml` leaves this at `none`.

#### and on a robot, where none of that exists

Three sources, mutually exclusive, and which one you have depends on the room:

| | |
|---|---|
| `optitrack_topic` / `mocap_pose_topic` | a lab with cameras |
| `odom_topic` | **an onboard estimator** — `docker/estimator` runs legged_control2's contact-aided Kalman filter over the robot's own IMU and encoders and publishes `nav_msgs/Odometry`. No cameras needed. x, y and heading DRIFT |
| `unitree_world_state` | the simulator only |

`run_difftrack_sim2sim.sh -E estimator` (**now the default**) exercises the
estimator path in sim, against the same container the robot runs. `-E compare`
scores the estimator against ground truth without letting it drive, and is the
first thing to run on a new clip.

Install and measurements:
[`docker/estimator/README.md`](../../docker/estimator/README.md) and
[`difftrack_state_estimation.md`](difftrack_state_estimation.md).

One trap that belongs here rather than there: under the unitree workflow the
simulator's ground truth and the estimator write **the same** `robot_state_`
fields, so `odom_topic` must be paired with `unitree_world_state:=none`. `-E
estimator` does that for you; by hand it is a launch argument, and forgetting it
produces a sim2sim result that looks good for a reason the robot does not have.

### what this plant does NOT share with mj_sim

A different number here is not a bug in either. The plants genuinely differ:

- **a different model** — unitree's own `g1_29dof.xml`, with its own armature
  (0.01), joint damping (0.05) and frictionloss (0.1–0.2);
- **a different integrator and timestep** — 2 ms Euler, MuJoCo's defaults, as
  unitree ships them;
- **a different place for the PD loop** — `unitree_mujoco` computes
  `tau = kp(q* - q) + kd(dq* - dq)` in its own 1 kHz bridge thread and writes
  torques into `ctrl`, where mj_sim uses position servos. This is the more
  faithful arrangement: it is what the robot's motor controllers do.

Consequences for the runner:

- **`state_decimation: 0`.** Under mj_sim, one `/robot_state` per physics step
  makes decimation an exact physics-per-control-step guarantee. `unitree_mujoco`
  publishes from a wall-clock thread that has no relationship to the physics
  thread, so the same trick guarantees nothing. What makes the physics right
  here is the simulator running at 1x — MuJoCo's `simulate` paces itself and
  shows the ratio it is achieving in its own panel. Watch that number.
- **no `-f`, no `-R`.** Both belong to `run_mj_sim.py`. The script refuses them
  under `-W unitree` rather than accepting them and doing nothing.
- **the camera follows, and that is the scene's doing, not the viewer's.** The
  viewer is stock MuJoCo `simulate`, whose free camera does not move — a G1 at
  1 m/s leaves the frame in about four seconds. The generated scene adds a
  `track` camera (mode `trackcom`) and selects it as the initial camera via
  `<visual><global cameraid="...">`, which `simulate`'s `AlignAndScaleView()`
  reads at load. It holds a constant offset from the robot's centre of mass, so
  the robot stays at one place on screen and only the background moves,
  whatever direction it walks and however far.

  Two things had to be true and only one was. Adding the camera without
  selecting it is a no-op — `cameraid` defaults to -1, the free camera — and a
  camera that is aimed wrong is worse than none: the shipped one pointed 26.6°
  down from 1.2 m at 3 m range, so its optical axis crossed the robot's plane
  at **z = −0.3 m, below the floor**, and framed a G1 with its head cut off
  over two thirds of empty ground. It is now solved from a look-at
  (`_chase_camera()` in `make_sim2sim_scene.py`) — a three-quarter **front**
  view from the robot's right, 2.8 m out, aimed at its middle — and tunable per
  run without a rebuild via `DRCL_CAM_AZ` / `DRCL_CAM_DIST` / `DRCL_CAM_EYE` /
  `DRCL_CAM_AIM`. `DRCL_CAM_AZ` is a bearing from dead astern: 0 over the
  shoulder, 90 profile, 125 (the default) three-quarter front, 180 head-on.

- **`F9` records an mp4**, and the rest of MuJoCo's keyboard works, but only
  with `workflows/patches/unitree_mujoco-f9-recording.patch` applied.
  `unitree_mujoco`'s `main.cc` installs its own GLFW key callback over
  `GlfwAdapter`'s, and GLFW allows exactly one — so on a stock build **`space`,
  `[`, `]`, Esc, F1 and Ctrl+P all do nothing**, which is why the initial
  camera above is selected in the scene rather than left to a keypress. The
  patch chains the callback back in and adds `F9`. See
  [`workflows/patches/`](../../workflows/patches/README.md) for the recorder's
  settings; `-R` drives it unattended.

---

## 2c. The stand cycle — the session an operator actually runs

Everything above engages the policy for you and measures one clip. This is the
other shape, and it is the one the robot gets: **the robot stands, you press
`A`, it plays the clip for a fixed time, and it comes back to standing.** Press
`A` again for another run.

```bash
cd $WS
bash src/cpp_control/scripts/run_difftrack_sim2sim.sh -s -d 10 g1_walk
```

Nothing is automated: `auto_engage` is off, nothing exits on its own, and the
viewer stays up until Ctrl-C. `-d` is the budget **per press**, not a run
length.

| button | |
|---|---|
| `A` | play the clip for `-d` seconds, then hand back to the rest state |
| `A` again | play it again, anchored to wherever the robot came to rest |
| `X` | nominal pose · `B` zeroing (limp) · `Y` damping |

With a gamepad on the simulator (`use_joystick: 1` in
`unitree_mujoco/simulate/config.yaml`) those are the pad's own buttons. Without
one, publish `/joy` from another terminal in the same environment — X-mode
indices, `A`=0 `B`=1 `X`=2 `Y`=3, and **only the rising edge counts**, so the
release matters:

```bash
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0,0,0,0,0,0], buttons: [1,0,0,0,0,0]}"
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0,0,0,0,0,0], buttons: [0,0,0,0,0,0]}"
```

### the rest state, which is the whole difficulty

**These tracking policies have no standing behaviour.** A clip is all they know,
and every shipped clip is a walk that starts and ends mid-stride at about
1 m/s. So something else has to hold the robot up on either side of one, and
what that something is decides whether the cycle works:

| `stand_onnx_path` | rest state | catches a robot still walking? |
|---|---|---|
| set | SONIC stand **policy** (`ControlMode::STAND`), actively balancing, gains from its own manifest | **yes** |
| unset | nominal-pose PD hold at the yaml's hold gains | **no** — holds a standing robot indefinitely, and nothing more |

Measured on `unitree_mujoco`, `g1_walk`, 8-10 s budget, pelvis height ten
seconds after the handover:

```
rest state              exit_hold  exit_ramp   outcome
nominal-pose hold        0.0        0.0        0.100 m   on the floor
nominal-pose hold        1.0        0.5        0.100 m   on the floor
nominal-pose hold        0.0        0.8        0.100 m   on the floor  (ramped to the nominal pose)
SONIC stand policy       1.0        0.5        0.080 m   on the floor
SONIC stand policy       0.0        0.0        0.785 m   STANDING, twice in a row
```

Two things fall out of that table, and both are the same lesson.

**1. The rest state has to be able to step.** No amount of joint-space
gentleness stops a humanoid carrying 1 m/s: not a PD hold, not a gain fade, not
ramping the joints to the nominal pose on the way out. The `-s` mode therefore
defaults to the SONIC stand and says so loudly if its weights are missing.

**2. Hand over on the tick the clip ends.** `exit_hold` (freeze the reference
and let the policy brake) and `exit_ramp` (fade the gains back) both sound
gentler and both **default to 0**, because every tick they spend is a tick the
robot is not being balanced by anything. `exit_hold` is actively harmful — speed
over a 6 s freeze went 0.86 → 1.31 → 0.86 → 1.11 m/s and the robot was down at
1.6 s. It does not brake, it flails; a tracking policy asked to stand still on a
mid-stride single-support frame is outside everything it saw. That is the same
finding `buildLeadIn()` records from the *entry* side: *"the policies have no
standing behaviour, so parking them in front of a slow reference topples the
robot in about a second."*

There is no gain jump to ramp away either, which is why 0 costs nothing: the
stand engine brings its own gains out of its manifest (kp 14-99), and they are
the same order as the tracking policy's trained ones.

### the weights are git-lfs

`models/tracker/sonic/g1_sonic_base.onnx` is 56 MB and LFS-tracked
(`.gitattributes` covers every `.onnx` under `models/`). An unfetched one is a
130-byte text **pointer** that exists, is readable, and is not a model — ONNX
Runtime reports `Protobuf parsing failed`, which reads as a corrupt export
rather than as a missing fetch. Both the node and the sim2sim script now check
for the pointer and say which it is.

```bash
cd $PKG
git lfs install --local
git lfs pull -I 'models/tracker/sonic/*'
bash workflows/conda_env/build.sh --packages-select cpp_control
```

Without it `-s` still runs, warns, and falls back to the nominal-pose hold —
which stands the robot up and holds it, and drops it the first time a clip ends
with the robot moving.

### the parameters

All of these are launch arguments on `g1_difftrack.launch.py`, so the cycle can
be driven by hand (§3) exactly as the script drives it.

| | default | |
|---|---|---|
| `start_in_stand` | `false` | enter the rest state on the first tick that has a real state message behind it, so the **first command the node ever publishes holds the robot**. Before this the choice was a constructor (`robot_state_` all zeros, so a ramp starts from a robot-shaped hole) or a timer (a tick of limp ZEROING reaching the robot first). Ignored under `auto_engage`, which owns the boot mode |
| `play_duration` | `-1` | seconds per `A` press. `<=0` is the whole clip, or forever if it loops |
| `stand_onnx_path` | `''` | a SONIC stand export to back the rest state. A bare relative name resolves under the installed `models/` |
| `exit_hold` | `0.0` | seconds with the reference clock frozen when the clip ends. **Measured harmful** — see above |
| `exit_ramp` | `0.0` | seconds fading the gains back to the yaml's hold gains, ramping the joints to the nominal pose as it goes. For a rest state that is passive AND a clip that ends still |

`EXIT_HOLD` and `EXIT_RAMP` in the environment override the script's own.

### on hardware

`start_in_stand` drives the joints from the node's first command. That is right
in simulation and it is a decision on a robot: it means the robot is held from
the moment the controller sees it, rather than sitting limp until someone
presses `X`. It is **not** `auto_engage` — nothing engages the tracking policy
without a button — but bring it up on the E-stop the first time.

The rest of §4 is unchanged, and its step 3 ("`X` — nominal pose. **Stop here
and look at it.**") is exactly the check that decides which rest state you can
use: the hold gains in the shipped yaml were measured in simulation.

---

## 3. sim2sim, by hand

What the script does, spelled out, for when you want to change something it does
not expose. **The unitree plant is first**, because it is the one that runs;
the mj_sim recipe below it is kept with §2's numbers.

### 3a. the unitree plant, by hand

```bash
cd $WS
source src/cpp_control/workflows/conda_env/runenv.sh   # sets the CycloneDDS middleware
export DISPLAY=:1; unset WAYLAND_DISPLAY XDG_SESSION_TYPE

# once: a scene outside the upstream checkout, with a `track` camera selected
SCENE=$(python3 src/cpp_control/scripts/make_sim2sim_scene.py \
          --flavor unitree --out /tmp/difftrack_scene --quiet | tail -1)
```

**Terminal 1 — the controller, first.** Wait for `difftrack loaded`.

```bash
CFG=$(ros2 pkg prefix cpp_control)/share/cpp_control/config/tracker/g1_difftrack_unitree.yaml
MODELS=$(ros2 pkg prefix cpp_control)/share/cpp_control/models

ros2 launch cpp_control g1_difftrack.launch.py motion:=g1_walk \
    config_path:=$CFG \
    start_in_stand:=true play_duration:=10.0 \
    stand_onnx_path:=tracker/sonic/g1_sonic_base.onnx \
    entry:=pose anchor_yaw_to_robot:=true observe_in_reference_frame:=true
```

**Terminal 2 — the simulator.** The MuJoCo window opens here.

```bash
env -u LD_LIBRARY_PATH $UNITREE_MUJOCO/simulate/build/unitree_mujoco \
    -r g1 -t 1 -c -i $ROS_DOMAIN_ID -n lo -s $SCENE
```

`env -u LD_LIBRARY_PATH` is not optional: `unitree_mujoco` is a plain C++ program
built against the SYSTEM toolchain, and `runenv.sh` has just put
`$CONDA_PREFIX/lib` at the front of the path. It then loads conda's libstdc++
and libyaml-cpp while its boost still resolves to the system one, and dies with
`free(): invalid pointer` moments after the DDS bridge starts — which reads as
the simulator crashing on our traffic, and is nothing of the kind. `-c` holds
the physics at the reset pose until the first `LowCmd`, so the robot does not
integrate limp while the bridge comes up.

**Terminal 3 — press `A`** (§2c), or use a gamepad.

### 3b. the drcl plant, by hand (RETIRED — see §2)

```bash
cd $DRCL          # the retired drcl/ workspace root -- not this tree
source cpp_control/workflows/conda_env/runenv.sh
export DISPLAY=:1; unset WAYLAND_DISPLAY XDG_SESSION_TYPE   # mj_sim needs X11

# once: a scene with the world_root weld OFF (the shipped one hangs the robot
# in the air for bring-up, and anything a weld holds up is not a result)
python3 cpp_control/scripts/make_sim2sim_scene.py --out /tmp/difftrack_scene --quiet

# once: an mj_sim config pointing at that scene
python3 - <<'PY'
import os, yaml, mj_sim
stock = os.path.join(os.path.dirname(mj_sim.__file__), 'config', 'G1.yml')
cfg = yaml.safe_load(open(stock))
cfg['sim']['model_path'] = 'g1/scene_flat_ground.xml'
yaml.safe_dump(cfg, open('/tmp/difftrack_scene/G1.yml', 'w'), sort_keys=False)
PY
```

**Terminal 1 — the controller, first.**

```bash
ros2 launch cpp_control g1_difftrack.launch.py motion:=g1_walk \
    entry:=pose auto_engage:=true play_duration:=-1.0
```

Wait for `difftrack loaded`. That wait is the entire ordering rule, and it is
the one thing easy to skip by hand.

**Terminal 2 — the simulator.** The MuJoCo window opens here.

```bash
SIM_ASSETS_PATH=/tmp/difftrack_scene \
    python3 cpp_control/scripts/run_mj_sim.py --cfgpath /tmp/difftrack_scene/G1.yml
```

`auto_engage:=true` walks the mode FSM through `X` then `A` by itself, which is
why no third terminal is needed. It is a **simulation convenience**: it drives
the joints from the node's first command with nobody having pressed anything.
Never use it on hardware.

### driving it yourself

Drop `auto_engage` and the buttons are yours: `X` nominal pose → `A` track
(`A` again restarts) · `B` zero · `Y` damp. With no gamepad, publish `/joy`
directly — X-mode indices, `A`=0 `B`=1 `X`=2 `Y`=3:

```bash
# X — hold the nominal pose. Ctrl-C after a second.
ros2 topic pub -r 10 /joy sensor_msgs/msg/Joy "{axes: [0,0,0,0,0,0], buttons: [0,0,1,0,0,0]}"
# A — track
ros2 topic pub -r 10 /joy sensor_msgs/msg/Joy "{axes: [0,0,0,0,0,0], buttons: [1,0,0,0,0,0]}"
```

Only the rising edge counts: a held button is one press, and the same button has
to be released — publish all-zero `buttons` — before it can be pressed again.

If you want to start from the **shipped** scene instead (weld active, robot
hanging), the order is `X` first, then `Y` in the MuJoCo window to drop it, then
`space`, then `A`. `Y` before `X` drops a limp robot.

---

## 4. On the real robot

Everything above the plant is identical — same node, same export, same buttons.
Three things change, and all three are load-bearing.

### 4.1 what changes

| | sim (drcl) | sim (unitree, §2b) | hardware |
|---|---|---|---|
| config | `g1_difftrack.yaml` | `g1_difftrack_unitree.yaml` | **`g1_difftrack_hw.yaml`** |
| `workflow` | `drcl_deploy` (G1State/G1Command) | `unitree` (unitree_hg LowState/LowCmd) | `unitree` (unitree_hg LowState/LowCmd) |
| topics | `/robot_state`, `/robot_command` | `/lowstate`, `/lowcmd` | `/lowstate`, `/lowcmd` |
| `state_decimation` | `10` — the control loop ticks off the state stream, so the policy gets exactly 10 x 2 ms of physics per step whatever the simulator's pacing does | `0` — the bridge publishes on a wall clock unrelated to the physics thread, so decimation would guarantee nothing | **`0`** — the plant is real time and its own clock is the authority; the 50 Hz wall timer is correct |
| world pose | on the state message | `unitree_world_state: sportmode_imu` | **OptiTrack**, separately |
| `auto_engage` | fine | fine | **never** |

The middle column exists so that the only difference left between what you have
tested and what you take to the robot is the plant and the world pose. Run it
before §4.

### 4.2 build requirements

Two optional packages, both checked at configure time. Read the output:

```
-- Found unitree_hg -- Unitree HG backend enabled
-- Found optitrack_msgs -- g1_difftrack_node OptiTrack input enabled
```

- **`unitree_hg`** — without it, `workflow: unitree` throws
  `Requested workflow backend not compiled` at startup. That is the right
  failure and it happens before the node touches the robot, but find out at
  your desk rather than in the lab.
- **`optitrack_msgs`** — the lab's interface package
  (`crl-humanoid-ros/crl_optitrack_ros/optitrack_msgs`). It is a pure message
  package, so build it into this workspace:

  ```bash
  ln -s /path/to/crl-humanoid-ros/crl_optitrack_ros/optitrack_msgs $WS/src/optitrack_msgs
  cd $WS && bash src/cpp_control/workflows/conda_env/build.sh --packages-select optitrack_msgs
  bash src/cpp_control/workflows/conda_env/build.sh --packages-select cpp_control
  ```

  Build it into this workspace rather than reusing a `crl_ws` install: that
  workspace is ROS 2 **jazzy** and this one is **humble**, and you cannot link
  across them.

  Without `optitrack_msgs`, `optitrack_topic:=` raises at startup with an
  explanation instead of silently doing nothing, and `mocap_pose_topic` (plain
  `geometry_msgs/PoseStamped`) still works.

### 4.3 OptiTrack

The adaptor is not part of this package and has to be running first, on an
x86-64 PC on the same ROS network — it cannot run on the robot's aarch64 Jetson:

```bash
ros2 launch optitrack_adaptor remote.py
```

Check the stream from **the same environment the controller will run in**,
because this is the hop that crosses ROS distros:

```bash
source src/cpp_control/workflows/conda_env/runenv.sh
DRCL_ROS_NETWORK=1 CYCLONE_IFACE=<nic> source src/cpp_control/workflows/conda_env/runenv.sh
ros2 topic hz /optitrack_adaptor/mocap_frame
ros2 topic echo --once /optitrack_adaptor/mocap_frame | head -40      # note the rigid-body id
```

If nothing arrives here while it arrives on the jazzy side, do not fight it:
republish the pelvis pose as `geometry_msgs/PoseStamped` from a node in the
adaptor's own workspace and use `mocap_pose_topic:=` instead. Both inputs feed
the same estimator.

Three things to get right in Motive before this means anything:

- **The rigid body must be the pelvis.** Its origin and axes are what the policy
  reads as the robot's base. If the marker cluster's origin is not the pelvis
  frame, correct it with `optitrack_offset` — a translation **in the rigid
  body's own frame**, so it rotates with the robot (a cluster taped to the back
  of the pelvis is exactly this case).
- **Z is up and the ground plane is at z = 0.** The observation carries absolute
  root height, and the export's fall cut-out is an absolute height.
- **Note the rigid-body id.** It is what `optitrack_rigid_body_id` selects out
  of the frame, and a wrong one gives
  `OptiTrack frame carries N rigid bodies, none with id X` once every two
  seconds and no pose at all.

### 4.4 the launch

```bash
DRCL_ROS_NETWORK=1 CYCLONE_IFACE=<nic> source src/cpp_control/workflows/conda_env/runenv.sh

CFG=$(ros2 pkg prefix cpp_control)/share/cpp_control/config/tracker/g1_difftrack_hw.yaml

ros2 launch cpp_control g1_difftrack.launch.py \
    motion:=g1_walk \
    config_path:=$CFG \
    optitrack_topic:=/optitrack_adaptor/mocap_frame \
    optitrack_rigid_body_id:=<id> \
    optitrack_offset:="[0.0, 0.0, 0.0]" \
    entry:=pose entry_ramp:=0.0
```

Confirm these two lines before touching the gamepad:

```
difftrack world state: OptiTrack on /optitrack_adaptor/mocap_frame, rigid body id 1, offset [+0.000 +0.000 +0.000]
difftrack: first OptiTrack frame for rigid body 1 at [+0.412 -1.208 +0.731] m
```

The second one is the check that matters: the position must be where the robot
actually is, and the height must be the pelvis height (~0.7-0.8 m standing). A
plausible-looking wrong pose is the one failure mode with no symptom until the
policy acts on it.

### 4.5 the sequence

1. Robot on the ground, standing, in its own damping/zero state. Operator on the
   E-stop.
2. `B` — zeroing. The node is passive.
3. `X` — nominal pose. The robot ramps to the policy's default pose at the
   yaml's **hold** gains and holds it. **Stop here and look at it.** If it
   cannot hold this, nothing after it is safe: the hold gains in the shipped
   yaml were measured in mj_sim, on the simulated plant, and are a starting
   point on the robot, not a tuning.
4. `A` — track. The clip is anchored to wherever the robot is standing, in yaw
   and horizontal position, and the observation is built in the clip's frame, so
   the robot's heading on the floor does not matter.
5. `A` again restarts the clip. `Y` damps. `B` zeroes.

**Do not set `entry_ramp` above 0.** It ramps the joints onto the clip's first
frame before engaging, and every shipped clip starts mid-stride in single
support: statically posing a standing robot into that puts it on the floor
(measured in mj_sim — falls at step 0). Engaging straight from the nominal pose
and letting the policy find the clip is what works.

### 4.6 what the node refuses to do

These are guards, not diagnostics — each stops the run rather than reporting it:

- **No world pose** → will not engage:
  `difftrack cannot engage: no world base state`.
- **Stale mocap** → will not run. `mocap_timeout` (default 0.2 s) measures
  arrival, not the publisher's stamp. A frozen pose is indistinguishable from a
  stationary robot right up until the policy acts on it.
- **Both mocap sources set** → refuses at startup. Two sources interleaving into
  one velocity estimator is not a configuration anyone wants.
- **A joint name in the export that this node does not have** → fatal. A silent
  permutation is the one failure here that produces no error at all.
- **Root height below `fall_height`** → the run finishes and the node damps.

### 4.7 what is not verified here

Written from the code and from sim2sim; **not run on a robot**. Things to treat
as unverified until you have:

- **The hold gains.** `(300, 400, 300, 300, 60)` on (hip, knee, ankle, waist,
  arm) is a measurement of the simulated plant from its reset pose. Step 3 above
  is where you find out. §2b is a second, independent plant to check them
  against before the robot is one.
- **The humble ↔ jazzy DDS hop** to the OptiTrack adaptor. §4.3 is how you check
  it, and the PoseStamped republisher is the fallback.

What §2b **does** now cover, and what used to be on this list: the `unitree_hg`
message path itself. The LowCmd CRC, `mode_machine`, the hg motor order, the
CycloneDDS transport and `workflow: unitree` in the node are all exercised by
`-W unitree` against `unitree_mujoco`. A failure in any of them is now a
sim2sim failure at your desk instead of a surprise in the lab.

---

## 5. When it goes wrong

| symptom | cause |
|---|---|
| no window, controller just sits there | the launch is the controller only — start the simulator (§2, §3) |
| robot folds to the floor, limp | nothing is commanding it. Under `unitree_mujoco` that is the 716 ms DDS-bridge startup window (start the controller FIRST, and the simulator with `-c`; §2b), or a controller that crashed or exited. The script refuses to start the simulator on a controller that never reached `difftrack loaded`. *(On the retired mj_sim plant the cause was different — it drove `kp=0 kd=0.1` itself whenever `/robot_command` had no publisher.)* |
| fresh export changes nothing | the launches read `install/`. Rebuild |
| `expected exactly 1 publisher each on /lowstate and /lowcmd` | a stale simulator or controller survived an interrupted run — both ignore SIGTERM, so the script escalates to SIGKILL. A stale **controller** is the nastier one: it fights the live one for `/lowcmd`, the last message in wins, and if the loser is a node still in ZEROING the robot goes down limp while a healthy controller reports tracking. *(The retired drcl plant names the same row `/robot_state` and `/robot_command`.)* |
| the gait looks sped up, like a jog | under `unitree_mujoco`, check the ratio MuJoCo's own `simulate` panel reports — it paces itself to 1x and tells you what it is achieving. The clip itself is a 1.033 s cycle = 1.94 steps/s, a normal walk. *(`-f` and `run_mj_sim.py`'s pacing belong to the retired drcl plant; the script rejects `-f` here.)* |
| runs fine, tracks badly, in sim | under `unitree_mujoco` the shipped `state_decimation` is **`0`** — the bridge publishes on a wall clock unrelated to the physics thread, so decimation guarantees nothing and the simulator running at 1x is what makes the physics right. Watch `simulate`'s own ratio. *(`state_decimation: 10` is the retired mj_sim config's exact physics-per-control-step guarantee; §2b, §4.1.)* |
| `Requested workflow backend not compiled` | `workflow: unitree` without `unitree_hg` at build time (§4.2) |
| `optitrack_topic was set but this binary was built WITHOUT optitrack_msgs` | §4.2 |
| `OptiTrack frame carries N rigid bodies, none with id X` | wrong `optitrack_rigid_body_id`, or the robot is occluded |
| the simulator segfaults on start | Wayland. `export DISPLAY=:1; unset WAYLAND_DISPLAY XDG_SESSION_TYPE` for the X11 backend (the sim2sim script does this itself) |
| nodes on one machine cannot see each other | a VPN is eating DDS multicast. `runenv.sh` sets `ROS_LOCALHOST_ONLY=1`; use `DRCL_ROS_NETWORK=1` only when it genuinely has to cross the network |
| both processes healthy, zero messages delivered | different DDS middleware, domain or interface. Nothing errors — a subscription nobody publishes to is a normal state. The script sets all three; by hand it is `source runenv.sh` (which now selects CycloneDDS on `lo` by default) plus `-i $ROS_DOMAIN_ID -n lo` on the simulator |
| `no unitree_mujoco binary at ...` | [`workflows/unitree.md`](../../workflows/unitree.md) is not installed, or `$UNITREE_ROS2` points somewhere else (set in `workflows/conda_env/env.sh`) |
| `difftrack cannot engage: no world base state` | `unitree_world_state` is not `sportmode_imu`, or the simulator's `frame_pos`/`frame_vel` sensors are missing from the scene. Both halves must arrive — SportModeState has no orientation, LowState no position |
| the robot is only half in frame, head cut off | an OLD generated scene. The `track` camera used to aim 26.6° down at 3 m, i.e. below the floor; it is now solved from a look-at. Regenerate, or retune with `DRCL_CAM_AZ` / `DRCL_CAM_DIST` / `DRCL_CAM_EYE` / `DRCL_CAM_AIM` |
| the robot walks out of frame | you are on the free camera, which does not follow. The generated scene selects its `track` camera at load, so this means an OLD scene under `--out` — regenerate it. (`[` / `]` cycles cameras back only if the F9-recording patch is applied; without it unitree_mujoco eats every MuJoCo key) |
| `Protobuf parsing failed` loading a policy | the `.onnx` is an unfetched git-lfs POINTER, not a model. `git lfs install --local && git lfs pull` (§2c). The node and the sim2sim script both name this explicitly now |
| `-s`: the robot stands, plays the clip, then falls over shortly after | the rest state cannot catch a robot that is still walking. Check the `rest state` line in the controller's `difftrack loaded` block: `nominal-pose hold` cannot, `SONIC stand policy` can (§2c) |
| `-s`: the robot falls DURING the handover | `exit_hold` or `exit_ramp` is non-zero. Both default to 0 for a reason — every tick they spend is a tick nothing is balancing the robot (§2c) |
| `F9` does nothing, no mp4 appears | the patch is not applied to this `unitree_mujoco` build, or `ffmpeg` is not on its PATH. `-R` checks the binary for you and says so before the run |
| the mp4 is cropped, not scaled | `DRCL_RECORD_SIZE` is bigger than the scene's `<visual><global offwidth/offheight>`. The generated scene sets 1920x1080; a stock one is 640x480 |
| `-W drcl` exits with "needs the drcl_deploy workspace" | expected — `-W unitree` is the default. `mj_sim`/`messages`/`assets` went with the `drcl/` workspace (§2) |
| `stand_onnx_path is relative and no models_dir was given` | the node was started directly rather than through `g1_difftrack.launch.py`, which supplies `models_dir`. Pass an absolute path |
| the two workflows disagree on `mean_err` | expected. Different model, different integrator, and the PD loop lives in unitree_mujoco's bridge rather than in MuJoCo's actuators (§2b) |

---

## where things live

| | |
|---|---|
| the node | `src/tasks/tracker/g1_difftrack.cpp` |
| observation builder + anchor (ROS-free) | `include/common/g1/difftrack_obs.hpp` |
| the rest state + the A cycle | `enter_rest()` / `exit_control()` in `src/tasks/tracker/g1_difftrack.cpp`, and `on_first_state()` in `src/base.cpp` |
| the stand engine | `include/common/g1/sonic_stand.hpp`, wired by `G1Node` from `stand_onnx_path` |
| sim config (drcl, retired) | `config/tracker/g1_difftrack.yaml` |
| sim config (unitree) | `config/tracker/g1_difftrack_unitree.yaml` |
| hardware config | `config/tracker/g1_difftrack_hw.yaml` |
| launch | `launch/g1_difftrack.launch.py` |
| sim2sim: measured, `-w` watched, `-s` operator-driven | `scripts/run_difftrack_sim2sim.sh` |
| the simulator + follow camera (drcl, retired) | `scripts/run_mj_sim.py` |
| the simulator (unitree) | `$UNITREE_MUJOCO/simulate/build/unitree_mujoco`, via [`workflows/unitree.md`](../../workflows/unitree.md) |
| the F9 video recorder | `scripts/frame_recorder.py` |
| scene: weld cleared (drcl), or unitree's model + track camera (`--flavor unitree`) | `scripts/make_sim2sim_scene.py` |
| the unitree workflow's install | [`workflows/unitree.md`](../../workflows/unitree.md), and [`workflows/conda_env/README.md`](../../workflows/conda_env/README.md) for a machine without apt Humble |
| parity self-test | `tests/difftrack_selftest.cpp` |
| reference (export, observation, entry) | [`difftrack.md`](difftrack.md) |
| the exporter, in diffsimrl | `code/scripts/export_to_drcl_cpp_control.py` |
