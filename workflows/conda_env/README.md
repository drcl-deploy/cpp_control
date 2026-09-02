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
bash deps.sh                               # ONNX Runtime 1.22 + the `assets` package
bash build.sh --packages-select messages
bash build.sh --packages-select mj_sim
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
drcl/                       <- the colcon workspace root ($WS)
├── cpp_control/            <- this package (also symlinked into src/)
└── src/
    ├── messages/           git clone drcl-deploy/messages
    ├── mj_sim/             git clone drcl-deploy/mj_sim
    ├── assets/             git clone drcl-deploy/assets   (git lfs pull!)
    └── cpp_control -> ../cpp_control
```

`build.sh` derives `$WS` from its own location, so a different layout only needs
`WS=... bash build.sh`.

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

No source change to any package was needed for this: `cpp_control` already globs
`thirdparty/onnxruntime-linux-x64-*` for ONNX Runtime, and `deps.sh` only puts a
symlink there.
