# G1 onboard base-state estimator.
#
# Build context is the colcon workspace's src/ directory, because the image
# needs two things out of it:
#
#   unitree/unitree_hg              the LowState IDL — the SAME definition the
#                                   robot and unitree_mujoco publish. Built
#                                   from source rather than pulled from apt so
#                                   there is exactly one definition in play.
#   cpp_control/docker/estimator/ws the legged_odom package
#
#   docker build -f cpp_control/docker/estimator/Dockerfile -t g1-estimator .
#
# (docker/estimator/build.sh does that with the right context, from anywhere.)
#
# BASE IMAGE. qiayuanl/unitree:jazzy is legged_control2's own published image —
# ROS 2 Jazzy on Ubuntu 24.04 with legged_model, legged_estimation and
# unitree_description already installed, and the buildfarm apt sources already
# configured. Multi-arch (amd64 + arm64), so this same Dockerfile builds for a
# Jetson-based onboard computer. Pinning it by tag rather than digest is
# deliberate: the estimator tracks legged_control2's releases.
FROM qiayuanl/unitree:jazzy

SHELL ["/bin/bash", "-c"]

# CycloneDDS is NOT in the base image (it ships FastRTPS only), and it is not
# optional here: the robot's LowState comes from unitree_sdk2, which speaks raw
# CycloneDDS on `rt/lowstate`, and the rest of the deployment workspace is on
# CycloneDDS too. On FastRTPS this container discovers nothing and simply sits
# there publishing an estimate of a robot it cannot hear.
#
# rosidl_generator_dds_idl is unitree_hg's own build dependency — the same one
# unitree_ros2's README has you apt-install before its first colcon build. It
# emits the .idl the SDK side speaks, which is exactly why the messages built
# here stay wire-compatible with the robot.
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-rosidl-generator-dds-idl \
      ros-jazzy-nav-msgs \
      ros-jazzy-tf2-msgs \
      ros-jazzy-std-srvs \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /colcon_ws

COPY unitree/unitree_hg                     src/unitree_hg
COPY cpp_control/docker/estimator/ws/src    src

RUN source /opt/ros/jazzy/setup.bash \
 && colcon build --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      --packages-up-to legged_odom

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0

COPY cpp_control/docker/estimator/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "legged_odom", "g1_odom.launch.py"]
