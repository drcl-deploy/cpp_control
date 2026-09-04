# Onboard base-state estimation — install and run

How to give the difftrack controller a world-frame base pose when there is no
motion capture in the room: a Docker container running legged_control2's
contact-aided Kalman filter over the robot's own IMU and joint encoders,
publishing `nav_msgs/Odometry` on `/odom`.

Covers the simulator on your laptop and the real G1's onboard computer. Do the
simulator first — every step here is identical on the robot except the network
interface, and the robot is a bad place to find out that the image does not
build.

---

## TL;DR

```bash
WS=~/Documents/ETH/rsl/unitree_ros2/cyclonedds_ws
PKG=$WS/src/cpp_control

# 1. build the image (once; ~10 min, needs network)
bash $PKG/docker/estimator/build.sh

# 2. check it against the simulator's ground truth BEFORE letting it drive
cd $WS && bash src/cpp_control/scripts/run_difftrack_sim2sim.sh -E compare -d 15 g1_walk

# 3. run the policy on the estimate — this is now the default
bash src/cpp_control/scripts/run_difftrack_sim2sim.sh -d 15 -r 3 g1_walk
```

On the robot, step 1 becomes "load a saved image" and step 3 becomes the
hardware launch. Both are below.

---

## Why a container at all

`legged_control2` is a ROS 2 **Jazzy** binary distribution — closed source,
Ubuntu 24.04 debs from a private buildfarm. `cpp_control` builds against ROS 2
**Humble** in a conda environment. Those two cannot share a workspace, and
installing Jazzy system-wide next to what the robot already has is not something
to do to a robot you need working.

A container settles it. The estimator gets its own Jazzy userspace, the
controller keeps its Humble one, and the only thing crossing the boundary is a
DDS topic — which is transport, not ABI, and works across distros.

## What it is, and what it is not

```
  /lowstate  ─────────────►  ┌──────────────────────────┐
  (500 Hz, from the robot    │  container: legged_odom  │
   or unitree_mujoco)        │  · GmObserver → contact  │
                             │    wrench per foot       │
                             │  · LinearKalmanFilter    │
                             │    (contact-aided EKF)   │
                             └────────────┬─────────────┘
                                          │  /odom  nav_msgs/Odometry
                                          ▼
                             ┌──────────────────────────┐
                             │  host: g1_difftrack_node │
                             │  odom_topic:=/odom       │
                             └────────────┬─────────────┘
                                          │  /lowcmd
                                          ▼
                                       the robot
```

**It is a passive listener.** It subscribes to `/lowstate` and publishes
`/odom`. It never writes a command, never claims a ros2_control interface and
never opens the robot's SDK. `cpp_control` keeps sole ownership of `/lowcmd`.

That is a deliberate departure from how legged_control2 is normally deployed.
Upstream, the estimator is a `ros2_control` controller inside a
`controller_manager` that also owns the hardware interface — which is fine when
that stack drives the robot, and impossible here, because `cpp_control` already
does. Running both would put two writers on `/lowcmd`, and the loser of that
race is the robot. So this container links `legged_estimation` and
`legged_model` directly and skips the controller_manager entirely.

**The filter is not ours.** `legged::LinearKalmanFilter` and
`legged::GmObserver` are Qiayuan Liao's / Hybrid Robotics', and the G1
parameters in `ws/src/legged_odom/config/g1.yaml` are the ones BeyondMimic's
`motion_tracking_controller` runs on this robot. `legged_odom_node.cpp` is glue:
LowState in, `LeggedModel` filled, filter stepped, Odometry out.

## What it cannot do — read this before trusting it

There is **no absolute position reference** anywhere in the loop.

| quantity | observable? | why |
|---|---|---|
| height (z) | **yes** | a foot on the ground pins it |
| roll, pitch | **yes** | the IMU sees gravity |
| x, y | **no** | dead reckoning; drifts without bound |
| yaw | **no** | drifts with the IMU's own heading |

This is a property of the sensor set, not of the filter. Adding a camera or a
LIDAR is what fixes it, and legged_control2 supports exactly that — the filter's
`updatePositionMeasurement` takes an external position, and upstream feeds it
from `glim` LIDAR odometry. This node does not wire that up; there is no LIDAR
in this setup.

It is survivable for difftrack because a clip is short and is **anchored onto
the robot at entry**: what matters is drift accumulated over one clip, not
absolute accuracy. `-E compare` is how you find out what that number is on your
clip. Do not reuse this `/odom` as a world pose for anything that runs longer.

---

## Part 1 — the laptop / simulator

### 1.1 Prerequisites

Docker, usable without `sudo`:

```bash
docker run --rm hello-world
```

If that needs `sudo`, add yourself to the group once and log out and back in:

```bash
sudo usermod -aG docker $USER
```

### 1.2 Build

```bash
bash $PKG/docker/estimator/build.sh
```

Roughly 10 minutes on a cold cache: it pulls `qiayuanl/unitree:jazzy` (1.5 GB),
adds CycloneDDS, and colcon-builds `unitree_hg` plus `legged_odom`.

**The build needs `--network=host`, and `build.sh` passes it.** Not the usual
advice, and not optional here: a container on docker's default bridge inherits
the host's `/etc/resolv.conf` while being unable to reach the nameservers in it
whenever those belong to a VPN. The failure does not look like DNS — `apt`
prints `Ign:` for every source, burns a 60 s timeout on each, and then fails on
a package it "cannot find". If you see that, a VPN is up and the network mode
was dropped.

The build context is the **workspace's `src/`**, so the image compiles
`unitree_hg` from the same message definitions the rest of the workspace uses.
A narrower context is the one modification that produces a container which
discovers `/lowstate` and then deserialises it wrong.

### 1.3 Verify against ground truth — do this before anything else

The estimator's failure mode is a plausible number. A wrong twist frame, a
gravity sign, a permuted joint map: none of them raise, none of them stop the
robot standing, and every one of them shows up later as "the policy transferred
worse than it did in sim". The container can be up, publishing at 500 Hz, and
completely wrong.

So score it against the simulator's ground truth first:

```bash
cd $WS
bash src/cpp_control/scripts/run_difftrack_sim2sim.sh -E compare -d 15 g1_walk
```

The controller runs on **ground truth** — a known-good run — while the estimator
runs alongside untrusted, and
[`scripts/compare_odom_ground_truth.py`](../../scripts/compare_odom_ground_truth.py)
scores one against the other:

```
  pos_drift  m     mean      p95      max    final
  height_err m     ...
  lin_vel_err      ...
  yaw_err  rad     ...

  height error       0.0xx m early -> 0.0xx m late   (bounded — feet are pinning it)
  lin_vel_err vs heading   0.0xx facing +x -> 0.0xx turned  (no heading dependence)
```

Two of those lines are diagnostics rather than measurements, and they are the
reason to run this at all:

* **`height error … GROWING`** — the contact update is not being applied. Foot
  frame names, joint map, or a contact force threshold the robot never crosses.
* **`lin_vel_err vs heading … FRAME ERROR`** — the twist is being read in the
  wrong frame. It is near zero while the robot faces +x and large once it turns,
  which is why a static test cannot find it.

A gravity error is not subtle: `lin_vel_err` runs away at ~9.8 m/s per second
and `pos_drift` goes quadratic.

### 1.4 Run the policy on the estimate

`-E estimator` is the **default** — measuring against a pose no robot can
produce reports an upper bound, not a transfer result:

```bash
bash src/cpp_control/scripts/run_difftrack_sim2sim.sh -d 15 -r 3 g1_walk
bash src/cpp_control/scripts/run_difftrack_sim2sim.sh -s -d 10 g1_walk   # stand cycle
```

`-E ground_truth` restores the old behaviour, and it stays useful for one job:
separating a policy problem from an estimator problem. If a clip fails under
`estimator` and survives under `ground_truth`, the estimator is what changed.

Measured here, RSI entry, 15 s, 3 repeats per cell, same session:

```
                        ground truth                     estimator
policy            clean  mean_err        min_h     clean  mean_err        min_h
g1_walk            3/3   0.321 .334 .345  0.72-73   3/3   0.355 .372 .436  0.49-0.52
g1_walk__actlat    3/3   0.175 .183 .187  0.73-74   3/3   0.368 .369 .416  0.54-0.59
```

**12/12 clean** — nothing fell, either source, either policy. The estimator
costs ~+0.06 m of mean root error on `g1_walk` and 0.2 m of minimum pelvis
height; the robot crouches deeper mid-clip.

The second row is the one to read twice. `g1_walk__actlat` is nearly **twice as
good** as `g1_walk` on ground truth — and on the estimator that advantage is
entirely gone, both landing at 0.37-0.44. **The estimator is the bottleneck, not
the policy**, so a ranking measured on ground truth does not predict the
deployment ranking. That is why `-E estimator` is the default.

Both columns are optimistic about hardware for one reason worth keeping in
mind: **in simulation the IMU's yaw is exact**, and on the robot it drifts.

### 1.5 By hand

```bash
# terminal 1 — the estimator
bash $PKG/docker/estimator/run.sh -i lo

# terminal 2 — the controller
source $UNITREE_ROS2/setup.sh
ros2 launch cpp_control g1_difftrack.launch.py motion:=g1_walk \
    config_path:=$(ros2 pkg prefix cpp_control)/share/cpp_control/config/tracker/g1_difftrack_unitree.yaml \
    odom_topic:=/odom unitree_world_state:=none
```

`unitree_world_state:=none` is not decoration. The unitree sim config ships
`sportmode_imu`, which writes the simulator's ground-truth pose into the same
`robot_state_` fields the estimator writes. Leave both on and the base state is
whichever message arrived last — a plausible pose that is neither source.

---

## Part 2 — the real G1

The onboard computer usually has no internet and should not be building
container images. Build on the laptop, ship the image over.

### 2.1 Which computer, and which architecture

The G1's development computer is either an x86 module (**amd64**) or a Jetson
Orin (**arm64**). Check before building:

```bash
ssh unitree@192.168.123.164 uname -m      # x86_64 → amd64;  aarch64 → arm64
```

The base image is multi-arch and legged_control2 publishes both
`noble-jazzy-amd64` and `noble-jazzy-arm64` buildfarms, so either works — but a
`docker save` of an amd64 image will not run on a Jetson. For arm64 from an
x86 laptop, build with buildx (needs `qemu-user-static` on the laptop, once):

```bash
docker buildx build --platform linux/arm64 --network=host \
    -f $PKG/docker/estimator/Dockerfile -t g1-estimator:arm64 \
    --load $WS/src
```

### 2.2 Install Docker on the robot — this is the one step that needs sudo

On the robot, once:

```bash
ssh unitree@192.168.123.164
sudo apt-get update
sudo apt-get install -y docker.io
sudo usermod -aG docker $USER
# log out and back in, then:
docker run --rm hello-world
```

Nothing else on the robot needs `sudo`, and nothing else on the robot needs
internet.

### 2.3 Ship the image

```bash
# on the laptop
docker save g1-estimator | gzip > /tmp/g1-estimator.tar.gz     # ~600 MB gz
scp /tmp/g1-estimator.tar.gz unitree@192.168.123.164:/tmp/

# on the robot
gunzip -c /tmp/g1-estimator.tar.gz | docker load
docker images | grep g1-estimator
```

### 2.4 Copy the scripts

The image is self-contained; `run.sh` is not, and it is the thing that gets the
network right. Either copy the package across or just the one script:

```bash
scp -r $PKG/docker/estimator unitree@192.168.123.164:~/difftrack-estimator
```

### 2.5 Find the interface that carries LowState

**This is the step that goes wrong**, and it fails silently: the container comes
up clean, logs nothing wrong, and publishes an estimate of a robot it cannot
hear.

The robot's internal network is `192.168.123.0/24`. On the onboard computer the
interface facing the motor controllers is usually `eth0`; from an external
laptop it is whatever is cabled in.

```bash
ip -brief addr | grep 192.168.123      # → e.g.  eth0  UP  192.168.123.164/24
```

Then check LowState is actually on it, before the estimator is in the picture:

```bash
source ~/unitree_ros2/setup.sh          # sets RMW + CYCLONEDDS_URI
ros2 topic hz /lowstate                 # expect ~500 Hz
```

No output means the interface in `CYCLONEDDS_URI` is wrong, and the estimator
would have inherited exactly that.

### 2.6 Run it

```bash
bash ~/difftrack-estimator/run.sh -i eth0
```

`run.sh` uses `--network host`, which is required: docker's default bridge is a
separate broadcast domain and DDS discovery does not cross it.

Check it:

```bash
source ~/unitree_ros2/setup.sh
ros2 topic hz /odom                     # ~500 Hz
ros2 topic echo /odom --once            # pose.position.z ≈ 0.75 m standing
```

**`pose.position.z` is the sanity check that costs nothing.** A standing G1's
pelvis is about 0.75 m up. If it reads 0, or drifts away while the robot stands
still, stop: the contact update is not working and nothing downstream is worth
running.

### 2.7 Point the controller at it

```bash
source ~/unitree_ros2/setup.sh
ros2 launch cpp_control g1_difftrack.launch.py motion:=g1_walk \
    config_path:=$(ros2 pkg prefix cpp_control)/share/cpp_control/config/tracker/g1_difftrack_hw.yaml \
    odom_topic:=/odom
```

`g1_difftrack_hw.yaml` sets `unitree_world_state: none` already, so unlike the
sim config there is nothing to override — the robot has no ground truth to
conflict with.

Then follow the hardware bring-up order in
[`docs/trackers/difftrack_running.md`](../../docs/trackers/difftrack_running.md).
Nothing about that changes: the estimator only replaces where the world pose
comes from. In particular, **check the hold gains on the ground, in
NOMINAL_POSE, before anything else** — they are still a simulator measurement.

### 2.8 Start it on boot (optional)

```bash
docker update --restart unless-stopped g1-estimator
```

Only once it has been verified by hand. An estimator that restarts on boot also
restarts into a fresh origin, which is correct — but a broken one that restarts
forever is harder to notice than one that stays down.

---

## Reference

### `run.sh`

```
-i IFACE   network interface for CycloneDDS. `lo` for the simulator on this
           machine; the robot's own (e.g. eth0) on hardware. NOT optional on
           hardware: without it CycloneDDS guesses.
-d ID      ROS_DOMAIN_ID (default: $ROS_DOMAIN_ID, else 0)
-t TAG     image tag                    (default g1-estimator)
-n NAME    container name               (default g1-estimator)
-D         detached
-q         no container log on stdout
```

### Node parameters

`ws/src/legged_odom/config/g1.yaml`, and it is commented. The ones worth
knowing:

| parameter | default | |
|---|---|---|
| `six_dof_contact_names` | `[LL_FOOT, LR_FOOT]` | URDF frames. Six-DoF because a G1 foot is a sole, not a point |
| `joint_names` | 29, unitree_hg motor order | resolved by name against the URDF; an unplaceable name is fatal |
| `estimation.contact.force_threshold` | 150 N | on the momentum observer's estimated wrench. Roughly "this foot carries half the robot" |
| `publish_decimation` | 1 | one Odometry per LowState. Never above 10 |
| `nominal_dt` / `max_dt` | 0.002 / 0.05 | the step comes from the MEASURED interval; these are the fallback and the sanity bound |

### Re-zeroing

The filter starts at the origin and there is no absolute reference, so the
origin is wherever the robot was when the container started. To move it —
after picking the robot up, say:

```bash
docker restart g1-estimator
# or, without a restart:
ros2 service call /legged_odom/reset std_srvs/srv/Trigger
```

### Frames

Published `Odometry` is REP-105:

* `header.frame_id = odom` — pose is world-frame
* `child_frame_id = pelvis` — **twist is BODY-frame**

The controller's `odom_twist_frame` parameter defaults to `child` and rotates.
If you point `odom_topic` at a different publisher, check which contract it
honours: reading a body-frame twist as a world one is invisible while the robot
faces along +x and wrong everywhere else.

---

## Troubleshooting

| symptom | cause |
|---|---|
| build: `Ign:` on every apt source, then "cannot find package" | VPN DNS; the build needs `--network=host` (`build.sh` passes it) |
| container up, `/odom` silent | wrong `-i` interface, or `/lowstate` is not flowing. Check `ros2 topic hz /lowstate` first |
| `/odom` publishes, controller says it has no world state | `odom_topic:=` not passed, or QoS: the estimator publishes best-effort |
| controller: "set exactly one of mocap_pose_topic, optitrack_topic and odom_topic" | two world-pose sources; pick one |
| `pose.position.z` drifts while the robot stands still | contact update not applied — foot frames, joint map, or force threshold |
| position walks away during a clip | expected, and the reason for `-E compare`. Bounded drift over one clip is the design; unbounded is not |
| two `/odom` publishers | a container survived a previous run. `docker rm -f g1-estimator` |

## See also

* [`docs/trackers/difftrack_state_estimation.md`](../../docs/trackers/difftrack_state_estimation.md) — the design, the frame argument, and the measurements
* [`docs/trackers/difftrack_running.md`](../../docs/trackers/difftrack_running.md) — hardware bring-up order
* [legged_control2 documentation](https://qiayuanl.github.io/legged_control2_doc/) — the filter's own docs
* [HybridRobotics/motion_tracking_controller](https://github.com/HybridRobotics/motion_tracking_controller) — where the G1 parameters come from
