#!/bin/bash
# Container entrypoint: ROS + the overlay, then whatever was asked for.
set -e

source /opt/ros/jazzy/setup.bash
source /colcon_ws/install/setup.bash

# CYCLONEDDS_URI is passed in by run.sh. Without it CycloneDDS picks an
# interface by itself, which inside --network=host means it may well pick the
# wrong one and discover nothing.
if [ -z "$CYCLONEDDS_URI" ]; then
    echo "legged_odom: CYCLONEDDS_URI is unset — CycloneDDS will guess a network" >&2
    echo "             interface. If no /lowstate arrives, that is why." >&2
fi

exec "$@"
