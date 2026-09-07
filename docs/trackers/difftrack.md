# difftrack tracker

Deploying **diffsimrl** G1 motion-tracking policies (ADD/AMP, trained against a
differentiable Warp simulator) in `cpp_control`. One self-contained model
directory, consumed by `g1_difftrack_node`.

These are the same policies the lab's `crl-humanoid-ros` stack runs as its
DIFFTRACK state; this is a second, independent deploy of them. The observation
builder is a port of that one and is held to the same golden trace.

## background — how the export works

| stage | where | what |
|---|---|---|
| training | `diffsimrl`, `mode=train alg=forl_shac` + ADD | reference-conditioned tracking policy: `imitation.enable_tar_obs`, `imitation.global_obs` |
| graph | `export_to_crl_unitree.export_policy_onnx` | one ONNX graph, observation normalisation folded in; input `obs`, output `actions` |
| motion | `export_to_crl_unitree.export_motion` | the clip resampled onto the 50 Hz control grid and reduced to the `tar_obs` pieces, plus the reference pose per step |
| kinematics | `export_to_crl_humanoid_ros.export_fk` | MuJoCo's body tree for the 31 MimicKit bodies, so the controller can rebuild `char_obs` from joint encoders alone (self-tested against `data.xpos`, ~1e-15 m) |
| exporter | `diffsimrl/code/scripts/export_to_drcl_cpp_control.py` | writes the directory below, plus a golden rollout for the parity self-test |

```bash
# in the diffsimrl env
python scripts/export_to_drcl_cpp_control.py <run_dir> \
    --out <cpp_control>/models/tracker/difftrack --trace-steps 400
```

Full procedure, including which training runs qualify:
`diffsimrl/code/scripts/EXPORT_TO_DRCL_CPP_CONTROL.md`.

## the model directory

```
models/tracker/difftrack/<motion>/
    policy.onnx            actor; one Ort::Session::Run per 50 Hz control step
    motion.bin             raw little-endian float32: the reference clip
    difftrack_config.json  joint names, trained gains, torque limits, default
                           pose, action scale, obs layout, FK table, RSI frame
    trace.bin / trace.json the golden rollout (with --trace-steps)
```

`difftrack_config.json` is the authority, the way a `vibe.onnx.v1` manifest is
for the SONIC tracker: joint order, default pose and action scale come from it,
and the task yaml carries plumbing only. It is read with yaml-cpp (JSON ⊂ YAML),
so this adds no dependency the package did not already have.

The **one exception is the gains**, and it is deliberate — see "Two sets of
gains" below.

## the observation — 849 dims

```
char_obs  237 = root_height(1) + root_rot6(6) + root_lin_vel(3) + root_ang_vel(3)
                + joint_rot6(30 x 6) + dof_vel(29) + key_pos(5 x 3)
tar_obs   612 = 3 lookahead slots x 204, precomputed into motion.bin except the
                leading XY, which is (tar_root_xy - agent_root_xy)
```

**All of it is WORLD-frame.** These policies were trained with
`imitation.global_obs=true`, so the observation carries absolute root height,
absolute world orientation, and world-frame linear and angular velocity.

That is the single most important thing to know before running this on
hardware: **an IMU cannot supply it.** No position, no heading, no world linear
velocity. The node refuses to engage without a world state rather than feed the
policy a zero pose.

| where | source | how |
|---|---|---|
| mj_sim (`workflow: drcl_deploy`) | ground truth | `G1State.base_pose` / `base_twist`, from the pelvis `framepos`/`framequat`/`framelinvel`/`frameangvel` sensors — all WORLD frame, note that the angular velocity is *not* the body-local one a free joint keeps in `qvel[3:6]` |
| hardware | motion capture | `mocap_pose_topic:=/your/pose` (`geometry_msgs/PoseStamped`); the node finite-differences it for both velocities |
| unitree backend | — | `SportModeState` has velocity but no orientation, so it does not qualify on its own |

A mocap gotcha the node handles, and that is worth knowing about: velocity is
divided by the **measured** interval between messages, never a nominal frame
period. One dropped frame otherwise reports double the true speed straight into
a world-frame observation.

## joint order

Two orders, resolved by name at startup:

| order | who |
|---|---|
| the training MJCF's | `difftrack_config.json`'s `policy_joint_names`, the ONNX graph's action vector |
| the node's | `joint_names` in the task yaml — the unitree_hg motor order, which is also mj_sim's `command2actuator` map |

For the G1 these are the same permutation, and the node says so at startup
(`difftrack joint order: policy == motor order (identity)`). It resolves them by
name anyway and refuses a name it cannot place: a joint permutation is the one
failure in this pipeline that produces **no error at all** — the dimensions
match either way, and the robot merely looks drunk.

## two sets of gains

The node uses the checkpoint's trained gains **only while the policy is
driving**. Every other mode — zeroing, damping, the nominal pose, the entry ramp
— uses the yaml's.

This is not an oversight. The trained gains are BeyondMimic armature-derived and
soft (kp 14-99 on the shipped export); a policy actively commanding targets away
from the measured pose gets ample authority from them, but a static PD hold does
not. Holding a humanoid upright means paying the gravity torque about the ankle
without deflecting past the point of no return, and in mj_sim the G1 needs
(hip, knee, ankle) kp of about (300, 400, 300) to do it from the model's reset
pose — (200, 300, 200) falls, and so does the (100, 150, 40) the package's other
G1 yamls carry. The exact table is in `config/tracker/g1_difftrack.yaml`. With
one set of gains for both jobs the robot is on the floor before tracking
begins.

Startup prints both:

```
gains        policy kp 14-99 kd 0.9-6.3 | hold kp 60-400
```

## control pacing

**What ships is `state_decimation: 0`.** `g1_difftrack_unitree.yaml` and
`g1_difftrack_hw.yaml` both carry it: `unitree_mujoco`'s bridge publishes
`LowState` from a wall-clock thread with no relationship to the physics thread,
so decimating against it guarantees nothing, and on the robot the plant really
is real time. The rest of this section is the **retired mj_sim** plant, where
decimation was an exact physics-per-control-step guarantee — kept because it is
what the reference numbers in
[difftrack_running.md](difftrack_running.md) §2 were taken under.

`state_decimation: 10` in the retired `g1_difftrack.yaml` ticks the control loop
off the **state stream** rather than the wall clock: one control step every ten
`G1State` messages.

Upstream mj_sim does not run in real time — its loop is paced by an rclpy rate
at 1000 Hz against a 2 ms (500 Hz) physics step, so it steps as fast as it can
get to, measured 1.4x-1.8x here, and that moves with machine load.
(`scripts/run_mj_sim.py` paces to the wall clock instead and reports the
achieved factor; `--speed 0` restores the free-running behaviour.) A 50 Hz
wall-clock controller against a free-running plant then
hands the robot 40 ms of physics per 20 ms control step, i.e. the policy is
asked to control a plant running 1.7x fast. Since mj_sim publishes exactly one
state per 2 ms physics step, a decimation of 10 fixes the ratio at the 20 ms the
policy trained with, whatever the wall clock is doing.

On a plant that publishes one state per physics step, set it to
(state publish rate x control_dt); on `unitree_mujoco` and on hardware it is
`0`, the wall timer.

## usage

**Step-by-step, including hardware: [difftrack_running.md](difftrack_running.md).**
This section is the summary.

```bash
source unitree_ros2/setup.sh                 # or your own ROS 2 humble env

# 1. verify the export against the golden trace — no robot, no sim, no ROS
./build/cpp_control/difftrack_selftest \
    $(ros2 pkg prefix cpp_control)/share/cpp_control/models/tracker/difftrack/g1_walk

# 2. sim2sim — the script starts BOTH processes, in the order that keeps the
#    robot standing, and the viewer camera follows the robot
bash scripts/run_difftrack_sim2sim.sh -w -e stand g1_walk   # watch, until Ctrl-C
bash scripts/run_difftrack_sim2sim.sh -d 30 -r 3 g1_walk    # measure

# 3. by hand: CONTROLLER first, wait for `difftrack loaded`, THEN the simulator.
#    The launch file has no viewer in it; a simulator with no controller is a
#    limp robot on the floor.
ros2 launch cpp_control g1_difftrack.launch.py motion:=g1_walk \
    entry:=pose auto_engage:=true play_duration:=-1.0
SIM_ASSETS_PATH=/tmp/difftrack_scene \
    python3 scripts/run_mj_sim.py --cfgpath /tmp/difftrack_scene/G1.yml

# 4. hardware: different config, and a world pose that is not the robot's
ros2 launch cpp_control g1_difftrack.launch.py motion:=g1_walk \
    config_path:=$(ros2 pkg prefix cpp_control)/share/cpp_control/config/tracker/g1_difftrack_hw.yaml \
    optitrack_topic:=/optitrack_adaptor/mocap_frame optitrack_rigid_body_id:=1
```

Buttons: `X` nominal pose (the policy's own default pose) → `A` track from clip
frame 0 (`A` again restarts) · `B` zero · `Y` damp · `R1` the stand engine.

**The rest state.** These policies have no standing behaviour, so the node keeps
a resting controller either side of a clip: `start_in_stand` boots into it,
`play_duration` ends the clip on a clock, and `A` runs it again. Which engine
backs it is a config choice — `stand_onnx_path` gives the actively balancing
SONIC stand, unset gives the nominal-pose PD hold — and it is the difference
between the robot standing after a walk clip and the robot on the floor. The
measurements are in
[`difftrack_running.md` §2c](difftrack_running.md#2c-the-stand-cycle--the-session-an-operator-actually-runs).

**The arm handover** (`arm_blend`, off by default). A clip does not end where the
stand begins, so the handover is a step in the joint targets: `g1_dance30s` puts
the stand's first arm target 1.4-2.8 rad from where the arm actually is. The legs
need that step and the arms do not, so `arm_blend` interpolates the **arm
position targets** onto the stand over that many seconds, starting the instant
the stand takes the robot. The clip is untouched — it runs to its last frame with
the policy owning every joint — and gains, legs and waist go to the stand on the
handover tick either way. It starts from the arms' **measured** angles: a
tracking policy commands past the joint stops routinely, and interpolating from
one of those drives the joint the wrong way first. It does not decide whether the
robot stays up after a clip; see
[`difftrack_running.md`](difftrack_running.md#the-arm-handover-arm_blend) for
what does.

Launch arguments:

| argument | default | meaning |
|---|---|---|
| `motion` | `g1_walk` | directory under `models/tracker/difftrack/` |
| `model_dir` | — | absolute model dir; overrides `motion:=` |
| `entry` | `pose` | `pose` holds the nominal pose then engages; `direct` engages on the spot |
| `entry_ramp` | `0.0` | >0 also ramps the joints onto the clip's first frame — **measures badly**, see below |
| `anchor_motion_to_robot` | `true` | place the clip at the robot when tracking starts |
| `anchor_yaw_to_robot` | `true` | also turn it onto the robot's heading (only safe with the next one) |
| `observe_in_reference_frame` | `true` | build the observation in the clip's frame |
| `start_in_stand` | `false` | enter the rest state on the first state message, so the first command holds the robot |
| `stand_onnx_path` | `''` | SONIC stand export backing the rest state; relative names resolve under `models/` |
| `exit_hold` | `0.0` | seconds with the reference frozen at the end of a clip — **measured harmful** |
| `exit_ramp` | `0.0` | seconds fading the gains back to the hold gains at the end of a clip |
| `arm_blend` | `0.0` | seconds over which the **arm position targets** interpolate onto the SONIC stand once it has the robot. Arms only, positions only; the clip is untouched |
| `arm_blend_joints` | `[shoulder, elbow, wrist]` | substrings of `joint_names` it owns; add `waist` for the torso |
| `play_duration` | `-1` | seconds; ≤0 is the whole clip, or forever if it loops |
| `lead_in_duration` | `0.0` | ramp the reference from the robot's velocity onto the clip's |
| `fall_height` | export's | root height below which the run is called a fall |
| `optitrack_topic` | — | world state on hardware, straight off the lab adaptor (`optitrack_msgs/MocapFrameData`) |
| `optitrack_rigid_body_id` | `1` | which rigid body in the frame is the pelvis |
| `optitrack_offset` | `[0,0,0]` | added to the OptiTrack position, **in the rigid body's own frame** |
| `mocap_pose_topic` | — | world state on hardware from any `geometry_msgs/PoseStamped` source instead |
| `mocap_timeout` | `0.2` | seconds without a pose before the node refuses to run |
| `auto_engage` | `false` | drive the FSM without a joystick — **simulation only** |

## entry: how to start a clip

Every shipped clip starts **mid-motion** — `g1_walk` enters mid-stride at
0.97 m/s — and these policies have no standing behaviour of their own. So how
tracking starts matters more than anything else in this document.

Measured in mj_sim, 15 s of `g1_walk` (750 control steps), mean and max root
tracking error over the repeats:

| entry | clean | mean | max |
|---|---|---|---|
| `-e rsi` — robot starts on clip frame 0 | 3/3 | 0.312-0.319 m | 0.610-0.641 m |
| `-e stand` — engages from the nominal pose | 3/3 | 0.340-0.350 m | 0.793-0.815 m |
| `-e stand -p 2` — 2 s ramp onto clip frame 0 first | **0/2** | falls at step 0 | |

Over 30 s the RSI numbers *improve* — 0.283 and 0.296 m mean, 1500/1500 both
repeats — because the error is dominated by a start transient that then
amortises. The robot is briefly a metre behind (RSI cannot carry velocity across:
MuJoCo resets qvel to zero and the clip enters at 0.97 m/s), catches up, and
settles at a steady lag of roughly 0.22 m.

The ramp is the trap. Statically posing a standing robot into a single-support
mid-stride frame topples it before the policy ever runs, so `entry_ramp`
defaults to 0: hold the nominal pose, engage, and let the policy find the clip.
Costing 0.03 m of mean error over a full RSI start is a cheap way to start a clip
from a robot that is simply standing there.

`anchor_motion_to_robot` + `observe_in_reference_frame` are what make a standing
start work at any heading. The clip is placed onto the robot in yaw and
horizontal position, and the robot's state is mapped into the clip's own frame
before the observation is built. A yaw about gravity is an exact symmetry of the
plant and the observation is exactly equivariant under it, so the policy sees
the configuration it trained on whichever way the robot happens to be facing.

**The yaw anchor is not safe on its own.** With the observation left in the
world frame, a yawed clip asks a non-equivariant policy to extrapolate; the node
warns if you configure that combination. Keep `anchor_yaw_to_robot` and
`observe_in_reference_frame` together, or turn the robot onto the clip's heading
instead.

`lead_in_duration` synthesises an approach that starts at the robot's own
velocity and accelerates onto the clip's entry speed. It sounds like the fix for
the start transient above and is not: the same RSI run that is 3/3 clean at
lead-in 0 **falls at step 136** with a 0.5 s lead-in. (The same thing was
measured on the crl-humanoid-ros deploy of these policies.) Leave it at 0 unless
a specific clip measures better with it. It is also refused together with
`observe_in_reference_frame`, because the lead-in table is synthesised already
placed in the world and cannot be read back through an identity anchor.

## looping clips

`g1_walk` is MimicKit's one-second gait cycle with `LoopMode.WRAP`: it has no
last frame. Three mechanisms handle that, all resolved at export time:

1. **The table is a whole number of cycles *and* of control steps.** 1.0333 s is
   51.67 steps of 20 ms, so a one-cycle table wrapped at 52 runs the reference
   0.6 % slow. The exporter searches for the multiple that closes — three walk
   cycles = 3.1 s = 155 steps exactly.
2. **`wrap_delta`** is added to every root position before the anchor; getting it
   wrong teleports the reference 3.1 m every 155 steps. The export log prints it
   **per cycle** (≈1.036 m), the config stores it **per table repeat** (≈3.107 m).
3. **`play_duration`** gives the controller a budget, since there is no end to
   stop on. `-1` tracks the whole clip for a play-once one and forever for a
   looping one.

## the self-test

```bash
./build/cpp_control/difftrack_selftest <model_dir> [tolerance]
```

Replays the states diffsimrl actually visited through this stack's observation
builder and ONNX session, and scores both against what diffsimrl actually
produced. Run it after every export and every rebuild: it is the only check here
that catches a silent layout change, and it needs neither the robot, the
simulator, nor ROS.

Reference numbers for the shipped exports:

```
worst steady state  2.6e-07   (bound 2e-5)
worst loop repeat   2.1e-03   (bound 3e-3 — MimicKit's slerp, see below)
worst action        3.1e-06   (against an action RMS of 0.71)
```

Two bounds rather than one, because a looping clip cannot be held to a single
tolerance:

- Inside the clip table's first repeat the builder reads the very frames the
  exporter computed, so it is bit-identical. Past it, the builder reads step
  `k % clipSteps` while diffsimrl queried the motion library at absolute time
  `k*dt`. Same instant, different float32 computation.
- At **exact source-frame landings** (dt 0.02 s against a 120 fps clip lands
  every fifth step exactly twelve source frames on) float32 cancellation decides
  whether that reads as (frame j, blend 0) or (frame j-1, blend 0.99995), and
  MimicKit's slerp returns the *midpoint* of two near-identical quaternions
  whatever the blend. That is the 2.07e-3, it is flat over 30000 steps, and
  training was noised five times harder.
- At **the clip's seam** (`(k + lookahead) % clipSteps == 0`) the reference jumps
  from the clip's last frame to its first — g1_walk does not close perfectly.
  Reported, not scored: there is no answer that could match both sides.

The loop bound is still worth having: a wrong wrap — a missing per-repeat root
offset — shows up as **metres**, not milliradians.

## build

```
include/common/g1/difftrack_obs.hpp   observation builder + anchor + lead-in.
src/common/g1/difftrack_obs.cpp       ROS-free, MuJoCo-free, ORT-free: Eigen and
                                      yaml-cpp only, so the self-test can hold it
                                      to a golden trace on its own.
include/cpp_control/tasks/tracker/g1_difftrack.hpp
src/tasks/tracker/g1_difftrack.cpp    level-2 node over G1Node
config/tracker/g1_difftrack.yaml      plumbing + hold gains
launch/g1_difftrack.launch.py
tests/difftrack_selftest.cpp          parity against the golden trace
config/tracker/g1_difftrack_hw.yaml    hardware: workflow=unitree, wall-clock loop
scripts/make_sim2sim_scene.py         mj_sim scene with the robot on the GROUND
scripts/run_difftrack_sim2sim.sh      sim2sim, measured and -w watched
scripts/run_mj_sim.py                 mj_sim + a camera that follows the robot
```

`optitrack_msgs` is found the same way and is what compiles the OptiTrack input
in; without it the node still builds and `mocap_pose_topic` still works, but
`optitrack_topic:=` raises at startup rather than quietly doing nothing.

Eigen is found with `find_package(Eigen3 QUIET)` and only this task needs it —
everything else in the package uses plain arrays and `common/math_utils.hpp`.
Without Eigen the node drops out of the build with a message rather than making
Eigen a new requirement for every consumer.

## things that will bite you

- **The launches read the INSTALLED copy.** A fresh export under
  `models/tracker/difftrack/` does nothing until `colcon build` runs again.
- **mj_sim's shipped scene hangs the robot in the air.** `world_root` is a weld,
  `active="true"`, holding the pelvis at z = 1.0 — right for bring-up, fatal for
  a transfer result, and only clearable from the viewer (`Y`).
  `make_sim2sim_scene.py` generates a scene that never had it. If a run looks
  impossibly good, check this first.
- **Start the controller before the sim.** The node boots limp (ZEROING) and
  mj_sim resets the robot standing; every millisecond the sim runs unattended is
  the robot falling over. `run_difftrack_sim2sim.sh` orders them deliberately.
- **mj_sim's viewer segfaults under Wayland** (`libdecor-gtk` fails to init).
  Unset `WAYLAND_DISPLAY` and `XDG_SESSION_TYPE` to get the X11 backend.
  `--headless` is not an alternative: mj_sim's `reset()` unconditionally touches
  `self.viewer`, which headless never creates.
- **`auto_engage` is for simulation.** It drives the joints from the moment the
  node starts, with nobody having pressed anything.
- **mj_sim does not reliably die on SIGTERM**, and a survivor keeps publishing
  `/robot_state`. A controller fed two interleaved simulations produces numbers
  that look plausible and mean nothing, so `run_difftrack_sim2sim.sh` escalates
  to SIGKILL, waits for the graph to go quiet before starting, and asserts one
  publisher after each sim comes up. Ask the DDS graph directly for that count —
  `ros2 topic info` reports a cached publisher for minutes after its process is
  gone.
- **Do not edit a shell script while it is running.** bash reads a script
  incrementally, so rewriting `run_difftrack_sim2sim.sh` mid-sweep corrupts the
  running copy — it fails with a syntax error somewhere unrelated, or worse,
  silently runs half of each version.
- **Do not run two sweeps at once**, for the same reason as the stale simulator:
  they fight over `/robot_state` and over the generated scene.
- **The run is not bit-reproducible.** Two processes, independent timers; state
  pacing fixes the physics-per-control-step ratio, not the message timing. Take
  repeats.
