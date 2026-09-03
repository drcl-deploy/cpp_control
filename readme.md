# cpp_control

modular, multi-robot, multi-workflow C++ controller deployment for ROS2
(humble). design, figures and rationale: [docs/ethos.md](docs/ethos.md).

**where this lives.** a package inside `unitree_ros2/cyclonedds_ws/src/`,
alongside `unitree_hg`/`unitree_go`/`unitree_api` — one colcon workspace, one
`install/`, one overlay for a launch to read. it used to sit in a second
workspace (`drcl/`) and pick unitree_hg up across a prefix-path bridge; two
install trees holding the same package is exactly the "a fresh export changes
nothing" trap, and it cannot happen now.

the drcl_deploy plant (`mj_sim`, `messages/G1State`, the `assets` package) went
with that workspace. its code paths are still here, still conditional
(`find_package(messages QUIET)`), and still in git — reinstate the three
packages beside this one and they build again. what ships is the UNITREE path:
`unitree_hg` LowState/LowCmd over CycloneDDS, which is what the robot speaks.

## install — step by step

Everything lives under `unitree_ros2/`, and the colcon workspace is
`cyclonedds_ws` inside it. [workflows/unitree.md](workflows/unitree.md) is the
full recipe (MuJoCo, unitree_sdk2, unitree_mujoco); this is the package's own
part of it.

```bash
WS=~/Documents/ETH/rsl/unitree_ros2/cyclonedds_ws
PKG=$WS/src/cpp_control
```

**1. clone into `src/`**, beside the unitree message packages, so one
`colcon build` produces one `install/` and there is one overlay for a launch to
read.

```bash
cd $WS/src && git clone --recursive git@github.com:drcl-deploy/cpp_control.git
```

**2. onnxruntime** — [pre-built v1.22](https://github.com/microsoft/onnxruntime/releases/tag/v1.22.0),
symlinked under [thirdparty/](./thirdparty/). `workflows/conda_env/deps.sh` does
this for you.

```bash
ln -s /PATH/TO/onnxruntime-linux-x64-1.22.0 $PKG/thirdparty/
```

**3. policy weights** — every `.onnx` under `models/` is git-lfs-tracked. An
unfetched one is a 130-byte text POINTER that exists, is readable, and is not a
model; ONNX Runtime reports it as `Protobuf parsing failed`, which reads as a
corrupt export rather than as a missing fetch.

```bash
cd $PKG && git lfs install --local && git lfs pull
```

**4. build the message packages first, then this one.** `cpp_control`
deliberately leaves `unitree_hg` out of its `package.xml` (CMake picks whichever
backend is present at configure time), and colcon only puts a dependency's
prefix on the path when the package DECLARES it — so the order matters.

```bash
cd $WS
colcon build --symlink-install --packages-select unitree_go unitree_hg unitree_api
colcon build --symlink-install --packages-select cpp_control
```

On a machine without apt Humble — no root, or a different system distro — use
the conda toolchain instead; it handles the prefix path and the cmake flags:

```bash
bash src/cpp_control/workflows/conda_env/build.sh --packages-select cpp_control
```

Check the configure output. This is the line that decides whether the package
can talk to a G1 at all:

```
-- Found unitree_hg -- Unitree HG backend enabled
```

`find_package(... QUIET)` **caches its failure**, so the first build after
installing the messages needs `rm build/cpp_control/CMakeCache.txt`.

**5. optional extras**

| | |
|---|---|
| `optitrack_msgs` | the lab's OptiTrack interface, for hardware. Symlink it into `$WS/src/` and build it here — a `crl_ws` install is ROS 2 jazzy and this workspace is humble, and you cannot link across them |
| CLI shims (`publish-motion`) | `uv pip install -e $PKG --python venv/bin/python` |

**6. run anything** — one source per shell. It activates the env, sources the
overlay, and selects CycloneDDS on `lo` so a ROS 2 node sees `unitree_mujoco`'s
raw-DDS `rt/lowstate` as `/lowstate`:

```bash
source $PKG/workflows/conda_env/runenv.sh
```

Do **not** use `unitree_ros2/setup.sh` with the conda toolchain: as shipped it
sources `/opt/ros/foxy`, a `$HOME/unitree_ros2` path and an interface name that
are all probably wrong on your machine.

## usage

```bash
# G1 locomotion
ros2 launch cpp_control g1_locomotion.launch.py

# G1 SONIC tracker (exported vibe policy, manifest-driven)
ros2 launch cpp_control g1_sonic_tracker.launch.py \
    onnx_path:=/path/to/policy.onnx motion_path:=/path/to/motion.npz

# hardware workflow (sonic + vibe-sonic): controller never dies —
# boot with no clip (stand), stream motions, A starts each one
ros2 launch cpp_control g1_sonic_tracker.launch.py motion_path:=''
publish-motion /path/to/motion.npz     # stage -> press A -> track -> RB -> repeat

# G1 difftrack tracker (diffsimrl ADD/AMP tracking policies) -- see below

# G1 vibe-SONIC (vision): encoder first, then the node, then the smoke viewer
ros2 launch vision_encoders encoder.launch.py model:=theia-tiny
ros2 launch cpp_control g1_vibe_sonic.launch.py \
    onnx_path:=/path/to/model.onnx motion_path:=/path/to/motion.npz
ros2 run cpp_control attn_viewer.py        # attention rows, live in the terminal

# deploy infra self-test (+ optional smoke of a real export)
./build/cpp_control/deploy_selftest [policy.onnx [policy.manifest.json]]

# MiniPi
ros2 launch cpp_control mini_pi_locomotion.launch.py
ros2 launch cpp_control mini_pi_dive.launch.py
```

buttons (joy / unitree gamepad): `X` nominal pose · `A` policy · `B` zeroing ·
`Y` damping · `RB/R1` stand. Only the RISING edge counts, so a held button is
one press and the same button has to be released before it can be pressed again.

**stand mode**: set `stand_onnx_path` in any g1 config yaml (or pass it as a
launch argument) to a base-SONIC export and the robot gets an actively-balancing
SONIC stand (`ControlMode::STAND`) — no task code involved. `RB/R1` engages it
by hand; the difftrack tracker also uses it as its REST state.

## difftrack — step by step

The diffsimrl tracking task, and the one with a run-book. Exporting a policy lives in
the training repo (`diffsimrl/code/scripts/EXPORT_TO_DRCL_CPP_CONTROL.md`);
running one is [docs/trackers/difftrack_running.md](docs/trackers/difftrack_running.md).

```bash
cd $WS
source src/cpp_control/workflows/conda_env/runenv.sh
```

**1. verify the export** against its golden trace — no robot, no simulator, no
ROS. A failure here means nothing downstream is worth running.

```bash
./build/cpp_control/difftrack_selftest \
    $(ros2 pkg prefix cpp_control)/share/cpp_control/models/tracker/difftrack/g1_walk
# ... worst steady state 2.4e-07 (bound 2e-5) ... PASS
```

**2. run the stand cycle** — the session a robot actually gets. The robot
stands, you press `A`, it plays the clip for `-d` seconds, and it goes back to
standing. `A` again runs it again.

```bash
bash src/cpp_control/scripts/run_difftrack_sim2sim.sh -s -d 10 g1_walk
```

**3. or watch / measure it** instead:

```bash
bash src/cpp_control/scripts/run_difftrack_sim2sim.sh -w -e stand g1_walk  # viewer up
bash src/cpp_control/scripts/run_difftrack_sim2sim.sh -d 30 -r 3 g1_walk   # one line per run
bash src/cpp_control/scripts/run_difftrack_sim2sim.sh -d 15 -r 3           # every export
```

The script starts `unitree_mujoco` itself, so a MuJoCo window opens in every one
of these — there is no headless mode. The viewer opens already tracking the
robot, and `F9` starts and stops an mp4 into `recordings/` (`-R` does it
unattended, one file per run) — both need
[`workflows/patches/`](workflows/patches/README.md) applied to `unitree_mujoco`. It also defaults to `-E estimator`, which
runs the onboard state estimator in a container so the number means something on
a robot; build the image once with `bash docker/estimator/build.sh`, or pass
`-E ground_truth` to measure against the simulator's own pose instead.

**4. on hardware**, the same node and the same export — the config and the world
pose are what change:

```bash
ros2 launch cpp_control g1_difftrack.launch.py motion:=g1_walk \
    config_path:=$(ros2 pkg prefix cpp_control)/share/cpp_control/config/tracker/g1_difftrack_hw.yaml \
    optitrack_topic:=/optitrack_adaptor/mocap_frame optitrack_rigid_body_id:=1
```

### the rest state

**These tracking policies have no standing behaviour.** A clip is all they know,
and every shipped clip is a walk that starts and ends mid-stride at about 1 m/s.
So something else holds the robot up either side of one, and which one it is
decides whether the cycle works at all:

| `stand_onnx_path` | rest state | catches a robot that is still walking? |
|---|---|---|
| set | SONIC stand **policy**, actively balancing, gains from its own manifest | **yes** — standing at 0.785 m ten seconds after the handover |
| unset | nominal-pose PD hold at the yaml's hold gains | **no** — holds a standing robot indefinitely, and nothing more |

`-s` selects the SONIC stand automatically when its weights are fetched, and
warns and falls back when they are not. The handover happens on the tick the
clip ends: `exit_hold` and `exit_ramp` both default to 0 because every tick they
spend is a tick nothing is balancing the robot. The measurements are in
[docs/trackers/difftrack_running.md](docs/trackers/difftrack_running.md) §2c.

## docs

| doc | what |
|---|---|
| [docs/ethos.md](docs/ethos.md) | design: backends, three levels, control modes, structure, how to extend |
| [docs/onnx_policies.md](docs/onnx_policies.md) | the two ONNX serving stacks (legacy vs manifest) and when to use which |
| [docs/motion.md](docs/motion.md) | `g1::Motion` — the one reference-motion class |
| [docs/trackers/custom_sonic.md](docs/trackers/custom_sonic.md) | vibe.onnx.v1 manifest contract, export flow, tracker usage |
| [docs/trackers/difftrack.md](docs/trackers/difftrack.md) | diffsimrl tracking policies: export, world-state requirement, entry, sim2sim |
| [docs/trackers/difftrack_running.md](docs/trackers/difftrack_running.md) | **run-book**: sim2sim, the stand cycle, by hand, and hardware bring-up with OptiTrack |
| [docs/trackers/difftrack_state_estimation.md](docs/trackers/difftrack_state_estimation.md) | running difftrack with NO motion capture: what the onboard estimator can and cannot observe |
| [docker/estimator/README.md](docker/estimator/README.md) | **install**: the onboard-estimator container, on this machine and on the robot |
| [workflows/unitree.md](workflows/unitree.md) | installing the workflow: MuJoCo, unitree_sdk2, unitree_mujoco, this package |
| [workflows/conda_env/README.md](workflows/conda_env/README.md) | building without root, or beside another ROS distro |

## acknowledgements

this package deploys policies trained with, and stands on the ideas of:

- [beyondmimic](https://github.com/HybridRobotics/motion_tracking_controller) — motion tracking controller
- [textop-tracker](https://github.com/TeleHuman/Textop) — text-conditioned whole-body tracking
- [SONIC](https://github.com/NVlabs/GR00T-WholeBodyControl) — whole-body control
- diffsimrl — differentiable-simulator motion tracking (ADD/AMP); the difftrack
  task deploys its exports, and its observation builder is ported from the same
  policies' deployment in `crl-humanoid-ros`
