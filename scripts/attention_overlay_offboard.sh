#!/usr/bin/env bash
set -euo pipefail

# Run after `ros2 run cdr_tcp_bridge bridge_client.sh` in another terminal.
# This viewer uses that localhost-only domain; it never joins robot DDS.
export ROS_DOMAIN_ID="${BRIDGE_ROS_DOMAIN_ID:-71}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

exec ros2 run cpp_control attention_overlay_viewer.py "$@"
