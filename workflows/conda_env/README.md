# Building this workspace without root, or beside another ROS distro

The documented setup ([drcl_deploy.md](../drcl_deploy.md), [unitree.md](../unitree.md))
wants ROS 2 Humble from apt. These scripts build the whole toolchain into a
conda environment instead, which is what the difftrack work was developed and
tested against. **Nothing here is required if you already have the apt setup** —
use it and ignore this directory.

They exist for two situations:

- no root on the machine, so `apt install ros-humble-*` is not available;
- the system already has a *different* ROS distro (this machine has
  `/opt/ros/jazzy`), and mixing distros breaks colcon in confusing ways.

```bash
bash mkenv.sh                              # conda env 'drclros': ROS 2 Humble + toolchain
bash deps.sh                               # ONNX Runtime 1.22
bash build.sh --packages-select unitree_go unitree_hg unitree_api
bash build.sh --packages-select optitrack_msgs      # optional, hardware only
bash build.sh --packages-select cpp_control
```

Then, to **run** anything (new shell, bash or zsh):

```bash
source workflows/conda_env/runenv.sh
ros2 launch cpp_control g1_difftrack.launch.py motion:=g1_walk
```

`runenv.sh` activates the env, sources the workspace overlay for your shell, and
sets `SIM_ASSETS_PATH`. Without it `ros2` is whatever the system has, and you get
`OSError: Environment variable 'AMENT_PREFIX_PATH' is not set or empty` or
`package 'cpp_control' not found`.

`env.sh` (which `runenv.sh` builds on) is the **build** environment and
deliberately does not source the overlay: running `colcon build` with the
workspace's own `install/` already sourced causes trouble.

Both `mkenv.sh` and `deps.sh` are idempotent — re-run them whenever the checks
complain. In particular a later `conda install` can pull `empy` or `cmake` back
to a version that does not work; re-running `mkenv.sh` puts them back.

## Workspace layout these scripts assume

```
unitree_ros2/                       <- $UNITREE_ROS2
├── cyclonedds_ws/                  <- the colcon workspace root ($WS)
│   ├── src/unitree/                git clone unitreerobotics/unitree_ros2
│   │   ├── unitree_go/  unitree_hg/  unitree_api/
│   ├── src/cpp_control/            <- this package
│   └── src/optitrack_msgs ->       symlink into crl-humanoid-ros (optional)
├── unitree_sdk2/                   git clone unitreerobotics/unitree_sdk2
└── unitree_mujoco/                 git clone --recursive unitreerobotics/unitree_mujoco
```

`env.sh` derives `$WS` from its own location (four levels up:
`$WS/src/cpp_control/workflows/conda_env/`) and `$UNITREE_ROS2` as its parent,
so a different layout only needs `WS=... bash build.sh`.

**This used to be two workspaces.** `cpp_control` sat in a `drcl/` workspace
next to `messages`, `mj_sim` and `assets`, and reached across to
`unitree_ros2/cyclonedds_ws/install` for `unitree_hg`. That is gone: there is
one workspace, one `install/`, and one overlay for a launch to read. The
drcl_deploy plant went with it — `find_package(messages QUIET)` simply does not
find anything, CMake says `messages NOT found -- DRCL deploy backend disabled`,
and the `mini_pi` nodes drop out of the build. Reinstate the three packages
under `src/` and they come back.

## The unitree workflow on top of this

[unitree.md](../unitree.md) is written for apt Humble. Almost all of it still
applies here — only the two ROS-side steps change, because this env already has
what they install.

| unitree.md says | here |
|---|---|
| `sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp` | already in the env |
| `sudo apt install ros-$ROS_DISTRO-rosidl-generator-dds-idl` | `conda install -c robostack-staging ros-humble-rosidl-generator-dds-idl` — then **re-pin** `empy==3.3.4` and `cmake<4`, which that install pulls forward |
| `colcon build` the whole `cyclonedds_ws` | only the three message packages; CycloneDDS and `rmw_cyclonedds_cpp` come from the env. From `cyclonedds_ws`, with `env.sh` sourced: `colcon build --packages-select unitree_go unitree_hg unitree_api --cmake-args -DCMAKE_POLICY_DEFAULT_CMP0094=NEW "-DCMAKE_CXX_FLAGS=-include cstdint"` |
| download and build MuJoCo | the prebuilt `mujoco-3.3.6-linux-x86_64.tar.gz` into `~/.mujoco/`, then symlink it as `unitree_mujoco/simulate/mujoco`. It already contains `include/`, `lib/` and `simulate/`, which is all `unitree_mujoco`'s CMakeLists uses — no source build |
| clone cpp_control into `cyclonedds_ws/src` | **yes** — that is where it lives now. One workspace, one `install/`; the "a fresh export does nothing" trap needed two of them and there is only one |

`build.sh` puts every sibling install prefix in `$WS/install` on
`CMAKE_PREFIX_PATH` and says whether it found `unitree_hg`; a configure that
finds it prints `Found unitree_hg -- Unitree HG backend enabled`. It has to do
this explicitly even though `unitree_hg` is now a sibling in the same
workspace — colcon puts a dependency's prefix on the path only when the package
DECLARES it, and `cpp_control` deliberately leaves both backends out of its
`package.xml`. `find_package(... QUIET)` **caches its failure**, so the first
build after installing the messages needs `rm build/cpp_control/CMakeCache.txt`
or it silently keeps the old answer.

`runenv.sh` sets the middleware up for you, and it is no longer opt-in:
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=0` and a
`CYCLONEDDS_URI` pinned to `lo` — `unitree_ros2/setup_local.sh`, in other words.
That is what makes a ROS 2 node see `unitree_mujoco`'s raw-DDS `rt/lowstate` as
`/lowstate`. It used to be behind `DRCL_WORKFLOW=unitree` because a second
plant on a different RMW shared the shell; there is one plant now.
`DRCL_WORKFLOW=none source runenv.sh` leaves the middleware alone.

On the ROBOT the interface is a real NIC rather than loopback:

```bash
DRCL_ROS_NETWORK=1 CYCLONE_IFACE=enp3s0 source workflows/conda_env/runenv.sh
```

## Why each workaround is there

Everything below is an artifact of the environment, not of this repository. They
are collected here so the next person does not rediscover them one build at a
time.

| symptom | cause | fix |
|---|---|---|
| `rosidl_adapter` dies with `module 'em' has no attribute 'BUFFERED_OPT'`, then `'NoneType' has no attribute 'shutdown'` | RoboStack pulls empy 4.x; ROS 2 needs 3.3.4 | `pip install empy==3.3.4`, **after** the conda install that pulls 4.x (`mkenv.sh`) |
| `mj_sim` fails with `error: option --editable not recognized` | colcon builds an ament_python package with `setup.py develop --editable`, and setuptools 80 removed the `develop` command | `setuptools<80` (`mkenv.sh`) |
| `cpp_control` stops at `At least one of unitree_hg or messages is required`, with `messages` built | colcon only puts a dependency's install prefix on the path when the package *declares* it, and cpp_control deliberately leaves both backends out of package.xml | `build.sh` adds every sibling install prefix to `CMAKE_PREFIX_PATH` |
| `Could NOT find Python (missing Python_EXECUTABLE …)` | `cmake_minimum_required(VERSION 3.8)` leaves CMP0094 OLD, so FindPython searches by version across the machine instead of using this env | `-DCMAKE_POLICY_DEFAULT_CMP0094=NEW` |
| `uint8_t does not name a type` in generated message headers | this env's GCC is far newer than Humble targets, and GCC 13 stopped including `<cstdint>` transitively | `-DCMAKE_CXX_FLAGS=-include cstdint` |
| flags silently folded into `CMAKE_BUILD_TYPE` | zsh does not word-split unquoted variables | run the build under bash |
| `CONDA_BUILD: unbound variable` on activate | RoboStack's activation scripts are not `set -u` clean | the scripts use `set -eo pipefail`, not `-euo` |
| MuJoCo: `number of faces should be between 1 and 200000 in STL file … perhaps this is an ASCII file?` | the `assets` meshes are git-lfs pointers | `git lfs pull` in `src/assets` |
| mj_sim exits with `TypeError` on `None + '/...'` | `SIM_ASSETS_PATH` is unset | `source runenv.sh` |
| mj_sim segfaults on startup, after `libdecor-gtk-WARNING: Failed to initialize GTK` | GLFW takes the Wayland backend under a Wayland session and libdecor cannot load its GTK plugin | `unset WAYLAND_DISPLAY XDG_SESSION_TYPE` for the X11 backend (XWayland is fine) |
| `AMENT_PREFIX_PATH is not set or empty`, or the package is not found, with a traceback under `/opt/ros/jazzy` | the env and workspace overlay were not sourced in that shell | `source runenv.sh` |
| `_comps: assignment to invalid subscript range` (zsh) | colcon's zsh hooks call `compdef` before `compinit` has run | `runenv.sh` runs `compinit` first |
| nodes on this machine cannot see each other, `ros2 service list` hangs | a VPN or firewall is dropping DDS multicast on the real NIC | `runenv.sh` sets `ROS_LOCALHOST_ONLY=1` (opt out with `DRCL_ROS_NETWORK=1`) |
| `ros2 topic info` reports a publisher that does not exist | the ros2 daemon caches participants for minutes after the process is gone | ask the graph directly (`get_publishers_info_by_topic`), or `ros2 daemon stop` |
| `unitree_mujoco` and the controller both start, both look healthy, and not one message is delivered | they are on different DDS middlewares, domains or interfaces. Nothing errors: a subscription to a topic nobody publishes on is a normal state | `DRCL_WORKFLOW=unitree source runenv.sh`, and give the simulator the matching `-i $ROS_DOMAIN_ID -n lo` |
| `Requested workflow backend not compiled` at startup, from a config with `workflow: unitree` | cpp_control was configured before `unitree_hg` existed, and `find_package(QUIET)` cached the miss | `rm build/cpp_control/CMakeCache.txt` and rebuild |

No source change to any package was needed for this: `cpp_control` already globs
`thirdparty/onnxruntime-linux-x64-*` for ONNX Runtime, and `deps.sh` only puts a
symlink there.
