

## install

`unitree_ros2` is the home of the whole deploy stack — the simulator, the SDK,
the message packages and this controller, in one tree with one colcon workspace
inside it:

```
unitree_ros2/                       $UNITREE_ROS2
├── cyclonedds_ws/                  $WS — the colcon workspace root
│   ├── src/unitree/                unitree_go, unitree_hg, unitree_api
│   ├── src/cpp_control/            this package
│   ├── src/optitrack_msgs ->       symlink; the lab's OptiTrack interface
│   └── install/                    the ONE overlay every launch reads
├── unitree_sdk2/                   what unitree_mujoco links against
└── unitree_mujoco/                 $UNITREE_MUJOCO — the simulator
```

### unitree_ros2
* clone
```
git clone https://github.com/unitreerobotics/unitree_ros2
```
* dependendencies
```
# source your ros2 , if not done by .bashrc

sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
sudo apt install ros-$ROS_DISTRO-rosidl-generator-dds-idl
sudo apt install libyaml-cpp-dev
```
* build
```
cd unitree_ros2/cyclonedds_ws
colcon build 
```

* make sure to update unitree_ros2/setup.sh as per your system paths and network interface

  As shipped it sources `/opt/ros/foxy` and `$HOME/unitree_ros2`, and pins a
  `NetworkInterface` name that is probably not yours — both are wrong on most
  machines, and the failure is `AMENT_PREFIX_PATH is not set` or two healthy
  processes exchanging nothing. On a machine using the conda toolchain
  ([conda_env/](conda_env/README.md)) do not use it at all:
  `source cyclonedds_ws/src/cpp_control/workflows/conda_env/runenv.sh` sets the
  distro, the overlay, the RMW, the domain and the interface together.

### python venv

one venv for every python-side tool (launch helpers, exporters, viewers) — uv,
system-site-packages so ros2 python (rclpy, launch) stays importable, prompt
named without the dot-dir noise:

```
cd unitree_ros2
uv venv venv --python 3.10 --system-site-packages --prompt unitree
source /opt/ros/humble/setup.bash   # ros first, then venv
source venv/bin/activate
```

install python deps only in here (`uv pip install <pkg>`); the c++ stack needs
none of it.

### mujoco 

download [mujoco 3.3.6 release](https://github.com/google-deepmind/mujoco/releases/tag/3.3.6), and extract it to the `~/.mujoco` directory;

```
cd ~/.mujoco/mujoco-3.3.6
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

### unitree_sdk2

`unitree_mujoco` links against it — its `CMakeLists.txt` does
`find_package(unitree_sdk2 REQUIRED)` and looks in `/opt/unitree_robotics/lib/cmake`
by default. The repository ships the library and its CycloneDDS prebuilt, so
this is a configure and an install, not a build.

```
cd unitree_ros2
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2 && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install
```

Its own dependencies:
```
sudo apt install libglfw3-dev libspdlog-dev libboost-all-dev libfmt-dev libeigen3-dev
```

### unitree_mujoco
* clone
```
cd unitree_ros2/ 
git clone --recursive git@github.com:unitreerobotics/unitree_mujoco.git
```

* establish mujoco link and build

  The link must point at a tree that has `include/`, `lib/` **and** `simulate/`
  side by side — `unitree_mujoco` compiles MuJoCo's own `simulate` sources into
  its binary. The prebuilt Linux release tarball
  (`mujoco-3.3.6-linux-x86_64.tar.gz`) has all three, so if that is what was
  extracted above, the cmake/make/install step in the mujoco section is not
  needed.
```
cd unitree_mujoco/simulate/
ln -s ~/.mujoco/mujoco-3.3.6 mujoco

mkdir build && cd build
cmake ..
make -j4 unitree_mujoco
```

  `make -j4` on its own also builds `jstest`, a standalone joystick utility that
  does not compile against current `unitree_sdk2` (`xKeySwitchUnion` lost the
  members it uses). Naming the target skips it; the simulator itself is fine.

* patch, for humanoids

  `unitree_mujoco` steps physics on a robot nothing is commanding for as long as
  its DDS bridge takes to come up — 716 ms of simulated time, measured. A G1 is
  on the floor well before that, so anything measured against the stock
  simulator is partly a measurement of DDS startup.
  [`patches/`](patches/README.md) has the numbers and a three-line fix:
```
cd unitree_mujoco
git apply /PATH/TO/cpp_control/workflows/patches/unitree_mujoco-startup-window.patch
git apply /PATH/TO/cpp_control/workflows/patches/unitree_mujoco-f9-recording.patch
cd simulate/build && make unitree_mujoco -j4
```

  The second patch is `F9` video recording — and the fix that makes MuJoCo's
  own keyboard work at all. `main.cc` installs its GLFW key callback over
  `GlfwAdapter`'s and GLFW allows only one, so on a stock build `space`, `[`,
  `]`, Esc, F1 and Ctrl+P are all dead. [`patches/`](patches/README.md) has
  both and the recorder's settings.

### without root, or beside another ROS distro

If ROS 2 Humble from apt is not an option (no root, or the machine already has a
different distro), [conda_env/](conda_env/README.md) builds the same toolchain
into a conda environment, and documents what this workflow needs on top of it —
which of the steps above still apply and which the conda env already provides.

### cpp_control
* clone — into `src/`, beside the message packages, so the two are built by one
  `colcon build` into one `install/`
```
cd unitree_ros2/cyclonedds_ws/src
git clone --recursive git@github.com:drcl-deploy/cpp_control.git
```
* policy weights. Every `.onnx` under `models/` is git-lfs-tracked. An unfetched
  one is a 130-byte text POINTER that exists, is readable, and is not a model —
  ONNX Runtime reports it as `Protobuf parsing failed`, which reads as a corrupt
  export rather than as a missing fetch.
```
cd cpp_control
git lfs install --local
git lfs pull                                  # or -I 'models/tracker/sonic/*'
```
* build — message packages first, then this one
```
cd unitree_ros2/cyclonedds_ws
colcon build --symlink-install --packages-select unitree_go unitree_hg unitree_api
colcon build --symlink-install --packages-select cpp_control
```
  On a machine without apt Humble — no root, or a different system distro — use
  the conda toolchain instead of a bare `colcon build`; it handles the prefix
  path and the cmake flags:
```
bash src/cpp_control/workflows/conda_env/build.sh --packages-select cpp_control
```
  `cpp_control` deliberately leaves `unitree_hg` out of its `package.xml` (CMake
  picks whichever backend is present at configure time), and colcon only puts a
  dependency's prefix on the path when the package DECLARES it — so build the
  message packages first. A configure that found the backend says so:
```
-- Found unitree_hg -- Unitree HG backend enabled
```
  `find_package(... QUIET)` CACHES its failure, so the first build after
  installing the messages needs `rm build/cpp_control/CMakeCache.txt`.

* optitrack_msgs (optional, hardware only) — the lab's own interface package.
  Symlink it in rather than reusing a `crl_ws` install: that workspace is ROS 2
  jazzy and this one is humble, and you cannot link across them.
```
ln -s /PATH/TO/crl-humanoid-ros/crl_optitrack_ros/optitrack_msgs \
      unitree_ros2/cyclonedds_ws/src/optitrack_msgs
colcon build --symlink-install --packages-select optitrack_msgs
```

## usage

One source per shell, and it is **not** `setup.sh`: as shipped that file sources
`/opt/ros/foxy` and `$HOME/unitree_ros2`, and pins a `NetworkInterface` name —
all three are wrong on most machines, and on the conda toolchain it is wrong on
every one. `runenv.sh` sets the distro, the overlay, the RMW, the domain and the
interface (`lo`) together.

```
cd unitree_ros2/cyclonedds_ws
source src/cpp_control/workflows/conda_env/runenv.sh
```

* in terminal1, spawn simulation
```
env -u LD_LIBRARY_PATH $UNITREE_MUJOCO/simulate/build/unitree_mujoco -i 0 -n lo -r g1 -c
```
  No `sudo`: that is only needed to bind a real network interface, and this runs
  on `lo`. `sudo` would also drop the conda environment that was just sourced.

  `env -u LD_LIBRARY_PATH` is **not optional** under the conda toolchain.
  `unitree_mujoco` is a plain C++ program built against the SYSTEM toolchain,
  and `runenv.sh` has just put `$CONDA_PREFIX/lib` at the front of the path; it
  then loads conda's libstdc++ and libyaml-cpp while its boost still resolves to
  the system one, and dies with `free(): invalid pointer` moments after the DDS
  bridge starts — which reads as the simulator crashing on our traffic, and is
  nothing of the kind.

  `-c` (`--wait-for-cmd`) holds the physics at the reset pose until the first
  `LowCmd`, so the robot does not integrate limp for the 716 ms the DDS bridge
  takes to come up. It comes from [`patches/`](patches/README.md); without that
  patch applied, drop the flag and start the controller first.

> [!NOTE]
> in `./unitree_mujoco/simulate/config.yaml` , set `use_joystick: 1` for joystick to be detected.

  Keys in the viewer, with [`patches/`](patches/README.md) applied: `F9`
  start/stop video recording (mp4 into `DRCL_RECORD_DIR`) · `space` pause ·
  `[` / `]` cycle cameras · Esc free camera · backspace reset · `7`/`8`/`9`
  elastic band. Without the patches only backspace and the band keys work.

* in terminal2, spawn controller
```
cd unitree_ros2/cyclonedds_ws
source src/cpp_control/workflows/conda_env/runenv.sh
ros2 launch cpp_control g1_locomotion.launch.py
```

### difftrack (diffsimrl tracking policies)

This is the workflow the difftrack tracker ships on, and its sim2sim script
drives both processes itself:

```
cd unitree_ros2/cyclonedds_ws

# measure it — no joystick, one result line per run
bash src/cpp_control/scripts/run_difftrack_sim2sim.sh -d 30 -r 3 g1_walk

# watch it — viewer up, tracking until Ctrl-C
bash src/cpp_control/scripts/run_difftrack_sim2sim.sh -w -e stand g1_walk

# drive it — the robot STANDS, you press A, it plays the clip for 10 s and
# comes back to standing. Press A again for another run.
bash src/cpp_control/scripts/run_difftrack_sim2sim.sh -s -d 10 g1_walk
```

It starts `unitree_mujoco` with a generated scene and the controller with
`config/tracker/g1_difftrack_unitree.yaml`, and it does not need `sudo` — that
is only for binding a real network interface, and this runs on `lo`.

One thing this workflow cannot give the tracker for free is a **world pose**.
These policies observe absolute height, absolute yaw and world-frame velocity,
and `LowState` carries none of it. In simulation `unitree_world_state:
"sportmode_imu"` assembles it from `SportModeState` + the IMU, which under
`unitree_mujoco` are ground-truth MuJoCo sensors. On the robot they are not, and
the hardware config leaves that setting off and takes mocap instead. See
[../docs/trackers/difftrack_running.md](../docs/trackers/difftrack_running.md).

