

## install 

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

### python venv

An optional venv can host `publish-motion` and other Python development tools.
Use the OS Python ABI and system packages so ROS stays importable. Runtime
viewing scripts use `/usr/bin/python3`; they do not require this venv.

```
cd unitree_ros2
uv venv venv --python /usr/bin/python3 --system-site-packages --prompt unitree
source /opt/ros/$ROS_DISTRO/setup.bash   # select your installed ROS distribution
source venv/bin/activate
```

Install development dependencies here with `uv pip install <pkg>`; install
runtime viewer dependencies through the workspace installer or ROS/system packages.

### mujoco 

download [mujoco 3.3.6 release](https://github.com/google-deepmind/mujoco/releases/tag/3.3.6), and extract it to the `~/.mujoco` directory;

```
cd ~/.mujoco/mujoco-3.3.6
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

### unitree_mujoco
* clone
```
cd unitree_ros2/ 
git clone --recursive git@github.com:unitreerobotics/unitree_mujoco.git
```

* establish mujoco link and build
```
cd unitree_mujoco/simulate/
ln -s ~/.mujoco/mujoco-3.3.6 mujoco

mkdir build && cd build
cmake ..
make -j4
```

### cpp_control 
* clone
```
cd unitree_ros2/cyclonedds_ws/src
git clone --recursive git@github.com:drcl-deploy/cpp_control.git
```
* build
```
cd unitree_ros2
source setup.sh && cd ./cyclonedds_ws
colcon build --symlink-install --packages-select cpp_control
```

## usage 
* in terminal1, spawn simulation 
```
cd unitree_ros2
source setup.sh 
sudo ./unitree_mujoco/simulate/build/unitree_mujoco -i 0 -n lo -r g1 
```
> [!NOTE] 
> in `./unitree_mujoco/simulate/config.yaml` , set `use_joystick: 1` for joystick to be detected. 

* in terminal2, spawn controller 
```
cd unitree_ros2
source setup.sh 
ros2 launch cpp_control g1_locomotion.launch.py 
```

