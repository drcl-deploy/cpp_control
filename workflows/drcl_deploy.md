## install 

> [!IMPORTANT]
> **RETIRED.** This workflow's three packages — `drcl-deploy/messages`,
> `drcl-deploy/mj_sim` and `drcl-deploy/assets` — lived in a `drcl/` colcon
> workspace that `cpp_control` no longer sits in. The package is now
> `unitree_ros2/cyclonedds_ws/src/cpp_control` and builds against `unitree_hg`
> alone; CMake reports `messages NOT found -- DRCL deploy backend disabled` and
> the `mini_pi` nodes drop out of the build.
>
> Nothing here was deleted. `find_package(messages QUIET)` still guards every
> drcl code path, `run_difftrack_sim2sim.sh` still carries its plant, and
> cloning the three packages into `cyclonedds_ws/src/` beside this one brings
> the backend back — the reference numbers in
> [`../docs/trackers/difftrack_running.md`](../docs/trackers/difftrack_running.md)
> were all taken on it.
>
> The workflow that ships is [`unitree.md`](unitree.md).

make a ros2 workspace 
```
mkdir -p drcl_deploy/src
```

### messages 

```
cd drcl_deploy/src
git clone git@github.com:drcl-deploy/messages.git
```

### mj_sim

```
cd drcl_deploy/src
git clone git@github.com:drcl-deploy/mj_sim.git
```
* pre-`cpp_control` build
```
cd drcl_deploy/
colcon build --symlink-install
```

### cpp_control 
* clone
```
cd drcl_deploy/src
git clone --recursive git@github.com:drcl-deploy/cpp_control.git
```
* build
```
cd drcl_deploy/
source install/setup.sh
colcon build --symlink-install --packages-select cpp_control
```

### python venv

same pattern as [unitree.md](unitree.md#python-venv), at the workspace root:

```
cd drcl_deploy
uv venv venv --python 3.10 --system-site-packages --prompt drcl
source /opt/ros/humble/setup.bash && source venv/bin/activate
```

### without root, or beside another ROS distro

If ROS 2 Humble from apt is not an option (no root, or the machine already has a
different distro), [conda_env/](conda_env/README.md) builds the same toolchain
into a conda environment and documents every workaround it needs.

## usage 
* in terminal1, spawn simulation 
```
cd drcl_deploy/
source install/setup.sh 
ros2 run mj_sim main -- --cfgpath /ABSOLUTE/PATH/TO/drcl_deploy/src/mj_sim/mj_sim/config/G1.yml
```
* in terminal2, spawn joystick
```
cd drcl_deploy/
source install/setup.sh 
ros2 run joy joy_node
```
* in terminal3, spawn controller 
```
cd drcl_deploy/
source install/setup.sh 
ros2 launch cpp_control g1_locomotion.launch.py 
```
