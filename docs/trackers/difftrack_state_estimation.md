# difftrack without motion capture — onboard state estimation

The design note. For "how do I install and run it", see
[`docker/estimator/README.md`](../../docker/estimator/README.md).

---

## 1. The problem

The difftrack policies were trained with `imitation.global_obs = true`. Their
observation is world-frame throughout: absolute root height, absolute world
orientation, and world-frame linear and angular velocity. Something has to
supply that at 50 Hz, and the node refuses to engage rather than hand the policy
a zero pose.

Three sources have existed so far, and all three are simulator or lab fixtures:

| source | where it works |
|---|---|
| `G1State.base_pose` / `base_twist` | mj_sim's ground truth. Gone with that workspace |
| `unitree_world_state: sportmode_imu` | unitree_mujoco's SportModeState + IMU. **Ground truth in sim; drifting odometry on a robot** |
| OptiTrack / any `PoseStamped` | a lab with cameras |

A G1 in a room with no cameras has none of them. What it has is a pelvis IMU,
29 joint encoders, and two feet that spend most of a walk cycle on the floor.
That is enough for height and tilt, and not enough for position and heading —
and the whole question is whether "not enough" is fatal here.

## 2. Why legged_control2, and why not the way it is normally used

[legged_control2](https://qiayuanl.github.io/legged_control2_doc/) ships a
contact-aided Kalman filter and a generalized-momentum observer that
[BeyondMimic's motion_tracking_controller](https://github.com/HybridRobotics/motion_tracking_controller)
runs on this exact robot for this exact class of policy. Reimplementing that
would mean reproducing someone else's tuned filter from a documentation page.

The catch is how it is packaged. `legged_controllers/StateEstimator` is a
**`ros2_control` chainable controller**. It runs inside a `controller_manager`
next to a hardware interface (`unitree_systems`) that owns the robot's SDK, and
it hands the base state to sibling controllers through an exported state
interface — `state_estimator/legged_model`, an in-process pointer. That is a
fine architecture when that stack drives the robot. Here it cannot be, for two
reasons:

1. **`cpp_control` already owns `/lowcmd`.** Bringing up `unitree_systems`
   alongside it puts two writers on the robot's command topic. The loser of
   that race is the robot.
2. **The base state never leaves the process** as anything a plain `rclcpp`
   node can read, except as `/odom` — which is published by
   `LinearKalmanFilterRos2`, not by the controller's interface export.

So this deployment uses the **libraries** and not the controller.
`legged_estimation` depends only on `legged_model`, which depends only on
pinocchio and urdfdom — no `rclcpp`, no `ros2_control`, nothing that wants to
own hardware:

```
ros-jazzy-legged-estimation  →  ros-jazzy-legged-model  →  ros-jazzy-pinocchio
                                                            ros-jazzy-urdfdom
```

`docker/estimator/ws/src/legged_odom` is ~350 lines of glue around them:
unitree_hg LowState in, `LeggedModel` filled, `GmObserver` then
`LinearKalmanFilter` stepped, `nav_msgs/Odometry` out. Everything numerical
happens inside legged_control2's `.so`.

It is a **passive listener**. It subscribes and publishes; it never commands.

### Why a container

`legged_control2` is ROS 2 **Jazzy** binaries (Ubuntu 24.04, closed source,
private buildfarm). `cpp_control` builds against ROS 2 **Humble** in a conda
environment. Those cannot share a colcon workspace. A container gives each its
own userspace and puts a DDS topic between them — transport, not ABI.

Cross-distro DDS works, and it is worth knowing what it looks like when it does:
the Jazzy side logs

```
Failed to parse type hash for topic 'ros_discovery_info' ... from USER_DATA '(null)'
```

once per Humble participant it discovers. Jazzy publishes a type hash in
discovery `USER_DATA` and Humble does not. Discovery still completes and the
topics still match. Those lines are not the problem when something is wrong.

## 3. The filter

```
  /lowstate  ──►  LeggedModel (pinocchio, G1 URDF, floating base)
   500 Hz          q = [ kf position | IMU quaternion | encoder q ]
                   v = [ kf velocity | IMU gyro       | encoder dq ]   (LOCAL frame)
                   τ = [ 0 (base)    | motor tau_est ]
                        │
                        ├──►  GmObserver ──► estimated wrench per foot
                        │                     (no joint accelerations needed)
                        │
                        └──►  LinearKalmanFilter
                                state  [ base position, base linear velocity,
                                         one contact point per foot ]
                                process    IMU specific force, rotated by attitude
                                measurement leg kinematics to each foot, weighted
                                            by a contact probability from the
                                            wrench magnitude
                        │
                        └──►  /odom   nav_msgs/Odometry
```

Attitude is **not estimated**. The pelvis quaternion is Unitree's own onboard
fusion and the gyro is body-local, which is already pinocchio's freeflyer
convention. Only position and linear velocity are filtered.

The G1 parameters — `base_name: pelvis`, `six_dof_contact_names: [LL_FOOT,
LR_FOOT]`, the noise densities — are copied from
`motion_tracking_controller/config/g1/controllers.yaml`. Everything not listed
there is legged_control2's own default and is left alone deliberately: those
are the values the closed binary was tuned with.

Six-DoF contacts rather than three: a G1 foot is a **sole**, not a point. It
carries a moment, and the filter's ZMP-within-the-footprint test is what stops
a foot rolling onto its toe from being treated as a fixed contact point.

## 4. What is observable, and what is not

| quantity | observable | why |
|---|---|---|
| height (z) | **yes** | a foot on the ground pins it |
| roll, pitch | **yes** | the IMU sees gravity |
| x, y | **no** | dead reckoning; drifts without bound |
| yaw | **no** | drifts with the IMU's own heading |

This is the sensor set, not the filter. The fix is an absolute reference —
legged_control2 supports one (`updatePositionMeasurement`, fed from `glim`
LIDAR odometry upstream), and there is no LIDAR here.

**Why difftrack survives it.** Two properties of this task, and neither is
luck:

* A clip is short. `g1_walk` is 1.03 s, looped.
* The clip is **anchored onto the robot at entry** and observed in the clip's
  own frame (`anchor_motion_to_robot`, `observe_in_reference_frame` — see
  `docs/trackers/difftrack.md`). Absolute world position is not what the
  observation carries; displacement from the anchor is.

So the quantity that matters is drift accumulated **during one clip**, not
absolute accuracy. A slow yaw drift moves the anchor between clips, which is
the same thing as the robot having turned a little.

Do not reuse this `/odom` as a world pose for anything longer.

## 5. Frames — the failure that produces a plausible number

The published `Odometry` is REP-105:

```
header.frame_id  "odom"     pose is world-frame
child_frame_id   "pelvis"   TWIST IS BODY-FRAME
```

and `cpp_control`'s `odom_twist_frame` (default `child`) rotates both halves by
`pose.orientation` on the way in.

This is the single most dangerous line in the integration, because **a
body-frame and a world-frame twist agree exactly while the robot is upright and
facing along +x** — which is every static test. The error appears only once the
robot turns, and appears as degraded tracking rather than as anything that looks
wrong.

It is the same class of bug as MuJoCo's free-joint `qvel[3:6]`, which is
body-local where Warp reports world; copying it raw cost this project a 4x
tracking-duration regression in its own sim2mujoco path and survived a t=0
parity check because the two conventions coincide at the reset pose.

`compare_odom_ground_truth.py` tests for it directly, by splitting the velocity
error on heading:

```
lin_vel_err vs heading   0.0xx facing +x  ->  0.0xx turned
```

An error that grows with heading is a frame error. One that does not is noise.

## 6. Validating it

The estimator's failure mode is a plausible number. A wrong twist frame, a
gravity sign, a permuted joint map: none raise, none stop the robot standing,
and each shows up weeks later as "the policy transferred worse than it did in
sim". So the rig grew a mode that scores the estimator without letting it
drive:

```bash
bash scripts/run_difftrack_sim2sim.sh -E compare -d 15 g1_walk
```

The controller runs on **ground truth** — a known-good run — while the estimator
runs alongside and `scripts/compare_odom_ground_truth.py` scores one against the
other. Two of its lines are diagnostics rather than measurements:

* **`height error … GROWING`** — the contact update is not being applied. Foot
  frame names, joint map, or a force threshold the robot never crosses. Height
  is observable, so a growing height error is never "drift".
* **`lin_vel_err vs heading … FRAME ERROR`** — §5.

A gravity error is not subtle: `lin_vel_err` runs away at ~9.8 m/s per second
and `pos_drift` goes quadratic.

`-E` then chooses what actually drives the policy:

| mode | controller reads | for |
|---|---|---|
| `estimator` (**default**) | `/odom` | what the robot will do |
| `ground_truth` | SportModeState + IMU | separating a policy problem from an estimator problem |
| `compare` | ground truth, estimator scored alongside | validating the estimator |
| `mixed` | per channel (`ODOM_MIX`): ground truth or the estimate, for `pos_xy` `pos_z` `vel_xy` `vel_z` | attributing an `estimator` failure to one channel |

`estimator` is the default because measuring against a pose no robot can produce
reports an upper bound, not a transfer result.

`mixed` exists because `estimator` swaps all four estimator-fed observation
channels at once. `scripts/odom_channel_mixer.py` assembles the world state
channel by channel (orientation and angular velocity are the IMU's either way —
the estimator does not filter them) and logs ground truth and the estimate side
by side, including on runs the estimate is driving, which `compare` cannot show.
The channel ablation built on it, its sweeps and its analysis live in
`recordings/estimator_ablation/`. Sweeps want `-H` (headless unitree_mujoco).

### Measurements

`g1_walk`, `unitree_mujoco`, RSI entry, 15 s.

**The estimator against ground truth** (`-E compare`, 12555 paired samples over
14 s, robot walking 14.11 m):

| | mean | note |
|---|---|---|
| height error | **0.015 m** | 0.0139 early → 0.0158 late — **flat**. Observable, and this is the line that says the contact update is working |
| horizontal velocity error | **0.11 m/s** | flat across the run, against ~1 m/s of walking |
| vertical velocity error | 0.28 m/s | the dominant part of the velocity error; the filter smooths the pelvis's per-step bob |
| position drift | **1.61 m over 14.11 m travelled** | 11%, or 0.115 m/s. Dead reckoning, as designed — ~0.12 m per 1.03 s clip |
| yaw error | ~0 | attitude is the IMU's, which is ground truth in sim. **On a robot this drifts and this number is not predictive** |

The flat height error is the most useful line here. A gravity-handling error
does not look like this: it runs away quadratically. (Seen during bring-up, on a
robot with no controller: z reached −68000 m, which is ½·9.81·t² for the 118 s
it had been running.)

**The policy driven by the estimator**, two policies × two sources, 3 repeats
each, same session:

```
                        ground truth                     estimator
policy            clean  mean_err        min_h     clean  mean_err        min_h
g1_walk            3/3   0.321 .334 .345  0.72-73   3/3   0.355 .372 .436  0.49-0.52
g1_walk__actlat    3/3   0.175 .183 .187  0.73-74   3/3   0.368 .369 .416  0.54-0.59
```

**12/12 clean.** Nothing fell, on either source, with either policy.

Two readings, and the second is the one that should change what you do next.

**The estimator costs roughly +0.06 m of mean root error on `g1_walk`** (0.33 →
0.39) and about 0.2 m of minimum pelvis height (0.72 → 0.51) — the robot
crouches noticeably deeper mid-clip. It stays up, and it is not free.

**But look at `g1_walk__actlat`.** On ground truth it is nearly twice as good as
`g1_walk` (0.18 against 0.33) — it is the best policy exported so far, and the
one the export doc now recommends. On the estimator that entire advantage is
gone: both policies land at 0.37-0.44, indistinguishable.

So **the estimator is the bottleneck, not the policy.** Two consequences:

* A policy ranking measured on ground truth does not predict the deployment
  ranking. `-E estimator` is the arbiter for anything destined for the robot,
  which is why it is the default.
* Effort spent training a better tracker is currently spent below the noise
  floor of the state estimate. The lever is the estimator: an absolute position
  reference (§4) is what would move it, and legged_control2 already supports one.

Two caveats on these numbers, both of which make them optimistic:

* **Yaw is ground truth here.** In sim the IMU quaternion is exact. On the robot
  it drifts, and yaw is what the anchor is resolved against.
* **`g1_walk` goes straight**, so `compare` reports *"the robot never turned far
  enough to test the frame"* — the §5 twist-frame check has never actually
  fired. A turning clip is still needed to validate that convention end to end.

## 7. Two sources is the other way to get it wrong

`robot_state_.base_*` has one writer. The node enforces that at both levels:

* `mocap_pose_topic`, `optitrack_topic` and `odom_topic` are mutually
  exclusive — setting two is fatal at startup.
* The **simulator's** ground truth is not one of those three: it comes in
  through `unitree_world_state: sportmode_imu` in the config yaml, which the
  sim config ships enabled. Leaving it on while `odom_topic` is set gives the
  base state two writers and no error — it becomes whichever message arrived
  last.

That is why `unitree_world_state` is now overridable from the launch line, and
why `-E estimator` passes `unitree_world_state:=none`. On hardware it does not
arise: `g1_difftrack_hw.yaml` already sets `none`, because the robot has no
ground truth to conflict with.

## 8. Latency

Two DDS hops that mocap did not have: LowState → estimator → `/odom` →
controller, on loopback or the robot's own interface. The estimator runs at the
state rate (500 Hz, `publish_decimation: 1`) and the control loop takes the
freshest message, so this is a staleness question, not a rate one — a few
milliseconds against a 20 ms control period.

`publish_decimation` should not go above 10: a 50 Hz publish is already at the
control period, which puts a whole control step of age on the newest sample.

The policies this ships with were trained with `action_latency_rand` and
`obs_latency_rand`, which is the margin this is spending.

## 9. The world-state guard — the estimator does not only drift

Drift was the expected failure (§4). The one that knocks a jumping policy over
is different: at the takeoff and landing of a jump the estimate's horizontal
position **teleports 1.3–2.4 m within 0.1–0.2 s** (30–66 m/s), its horizontal
velocity error spikes to ~2 m/s at the same moment, and the position error then
stays. Measured with `-E mixed` (every channel on the estimate, ground truth
logged alongside) on `g1_jumps21`, a policy that plays the clip 4/4 on ground
truth at 0.19 m and falls 6/6 on the estimator at steps 139–223: before the jump
the estimate is within 0.1–0.4 m; after it the policy sees itself 1–2 m off a
clip it has never been more than 0.40 m from, lunges after it and falls within
10–30 steps. Replaying the recorded sensor stream into the estimator reproduces
the jump open loop (1.39 m within 0.2 s on the jumps bag).

So the controller holds glitches out, on every clip, before the observation is
built (`WorldStateGuard`, `include/common/g1/difftrack_obs.hpp`):

* Each control tick it takes the robot's xy displacement **beyond the
  reference's own displacement**. At most `world_guard_max_m` of that may pass
  in any `world_guard_window_s`; the excess is held out as a persistent xy
  correction (the estimator's error does not come back on its own; a jump the
  other way cancels it). Height is untouched.
* While a correction is fresh (`world_guard_hold_s` after the last rejection)
  the horizontal velocity is held within `world_guard_vel_dev` of the
  reference's.
* External world-state sources only (odom, mocap), and new rejections only while
  the clip's clock runs. Reset at every engage.

The defaults come from ground truth, not from taste:

| over any 0.2 s window, robot xy motion beyond the reference | max |
|---|---|
| g1_dance15s / g1_dance30s / g1_fight (ground truth) | 0.19 / 0.29 / 0.19 m |
| g1_jumps / g1_run (ground truth) | 0.28 / 0.30 m |
| g1_run_bundle, 3.75 m/s (ground truth) | 0.39 m |
| **g1_jumps21 on the estimator** | **2.1 m p99, 2.6 m max** |

| parameter | default | |
|---|---|---|
| `world_guard` | `true` | `false` restores the raw estimate |
| `world_guard_max_m` | `0.5` | |
| `world_guard_window_s` | `0.2` | |
| `world_guard_vel_dev` | `0.5` | m/s; genuine deviations reach 1.4 m/s p99 on g1_run, so it applies only while a correction is fresh |
| `world_guard_hold_s` | `0.5` | the glitch's velocity error outlasts its position jump |

Every run log carries `world_guard_offset` (the xy correction; subtract it from
`root_pos` for what the policy saw) and `world_guard_active`; the metadata's
`world` block has the parameters and `guard_reject_ticks` / `guard_max_offset`;
the SUMMARY line ends in `guard_max=… guard_ticks=…`. `difftrack_guard_selftest`
holds the guard to both properties: motion at 0.42 m per 0.2 s beyond the
reference passes bit-for-bit, and a 2 m glitch leaks at most 0.5 m.

Measured, it is a safety net and not the fix. It rejected nothing on 187 clean
ground-truth runs of every clip (and on a fresh all-clip check through the odom
path). On the stock estimator it does hold the jump out — 0.9–2.7 m of
correction per run — but g1_jumps21 still falls 6/6: after the jump the stock
filter's horizontal velocity stays 1–2.5 m/s wrong for over a second, which
drags the position at a rate the budget allows, and height swings ±0.3 m. A
tighter or longer budget was evaluated offline and rejected: 0.5 m per 1 s
fires on 109 of the 187 ground-truth runs (g1_run lags its clip by ~1 m in the
first second from a stand), and post-detection containment made the corrected
state no closer to the truth. What removes the glitch is the estimator's own
contact model (§10).

## 10. Estimator parameters per clip

`docker/estimator/run.sh -c <file>` mounts a full `legged_odom` parameter file
instead of the image's `g1.yaml` — no rebuild. Its default is
`ws/src/legged_odom/config/g1_dynamic.yaml`, so that is what the robot runs, for
every clip; `-c stock` is the image's `g1.yaml`. `run_difftrack_sim2sim.sh` uses
`g1_dynamic.yaml` for `g1_jumps*`, `g1_run*` and `g1_fight*` whenever that file
exists and the stock `g1.yaml` for everything else;
`ESTIMATOR_PARAMS=stock|<file>` overrides the choice for every clip. The tuning
that produced `g1_dynamic.yaml` — replaying ground-truth sensor bags into the
container under candidate parameters and scoring against ground truth — lives in
`recordings/estimator_tuning/`.

`g1_dynamic.yaml` changes two contact values and nothing else:

| | `g1.yaml` | `g1_dynamic.yaml` |
|---|---|---|
| `contact.zmp_length_x` / `zmp_length_y` | 0.08 / 0.025 | 0.12 / 0.04 |
| `contact.sensor_noise_position` | 0.002 | 0.05 |

The ZMP-within-the-footprint test decides whether a foot is a fixed contact, and
the stock footprint is smaller than a G1 sole: a foot rolling onto its toe or heel
at takeoff and landing is dropped and the filter dead-reckons through the jump —
the teleport of §9. The footprint alone halves the replay errors but does not
change the closed-loop outcome (g1_jumps21 still 0/6); trusting the leg
kinematics less as well is what does.

Open loop, replayed on ground-truth sensor bags (stock → dynamic):

| | g1_jumps21 | g1_run |
|---|---|---|
| xy error jump within 0.2 s | 1.36 → 0.18 m | 0.14 → 0.06 m |
| drift over the clip | 1.04 → 0.14 m | 0.23 → 0.26 m |
| horizontal velocity error, rms | 0.66 → 0.12 m/s | 0.11 → 0.09 m/s |
| height error, max | 0.60 → 0.11 m | 0.16 → 0.09 m |
| vertical velocity error, rms | 0.48 → 0.19 m/s | 0.26 → 0.15 m/s |

Closed loop, stand cycle on the estimator, clean runs (mean root error):

| | g1_jumps21 | g1_run |
|---|---|---|
| g1.yaml, guard off | 0/6 | 3/6 |
| g1.yaml, guard on | 0/6 | 2/6 |
| g1_dynamic.yaml, guard off | 6/7 (0.23 m) | 6/6 (0.37 m) |
| **g1_dynamic.yaml, guard on** | **6/6 (0.23 m)** | **5/6 (0.38 m)** |

For scale, g1_jumps21 on ground truth: 4/4 at 0.19–0.22 m. The one fall in the
shipped configuration (g1_run, step 643) had no guard activity. Replay-only and
not validated: `sensor_noise_position: 0.1` with the same footprint scored better
still on the jumps bag.

## See also

* [`docker/estimator/README.md`](../../docker/estimator/README.md) — install and run, laptop and robot
* [`difftrack.md`](difftrack.md) — the policies, the export, the world-state requirement
* [`difftrack_running.md`](difftrack_running.md) — bring-up order on hardware
