#!/usr/bin/env bash
# Live real-machine deployment wrapper for Go2W 3D Navigation Stack (mode:=real).
#
# NOTE: This is a THIN bringup wrapper for the REAL machine (onboard OrinNX).
# It is NOT a bag-replay script — that is bringup/replay_lio.sh (which replays a
# recorded bag offline and references root_config_real.yaml for offline testing).
#
# This script is a recorded-alias for the live bringup and is provided as
# documentation of the exact command used to start the full stack on the real
# machine. There is no live machine in the dev environment to test against, so
# it only defines ROS env + prints the canonical `ros2 launch` invocation.
#
# On the real machine (onboard OrinNX):
#   1. Source /opt/ros/humble/setup.bash + the Nav3D install/setup.bash.
#   2. Confirm the `unitree_go2w` driver package is installed (onboard-only;
#      absent from the dev tree). It is launched with mode:=real.
#   3. Start livox_ros_driver2 for the Livox MID360 (publishes /livox/lidar +
#      /livox/imu). Do NOT start it when replaying a bag.
#
# Usage:
#   bash bringup/replay_lio_real.sh <path/to/saved.pcd>
#
# Example:
#   bash bringup/replay_lio_real.sh /media/lenovo/disk/planner_ws/maps/campus3.pcd

set -e

NAV3D_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_PATH="${1:-/media/lenovo/disk/planner_ws/maps/campus3.pcd}"

# --- Clean env (avoid stale ROS workspace pollution) ---
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH 2>/dev/null || true
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-18}

# --- Source ROS + Nav3D ---
source /opt/ros/humble/setup.bash
source "$NAV3D_ROOT/install/setup.bash"

if [ ! -f "$MAP_PATH" ]; then
  echo "ERROR: PCD map not found at $MAP_PATH" >&2
  echo "  Usage: bash $0 <path/to/saved.pcd>" >&2
  exit 1
fi

echo "============================================================"
echo " Go2W 3D Nav Stack — LIVE deployment (mode:=real)"
echo "============================================================"
echo "  Map:      $MAP_PATH"
echo "  Mode:     real (unitree_go2w driver + livox_ros_driver2 live)"
echo "  Localization: true (root_config_real.yaml -> relocation_localization.yaml)"
echo
echo "Start the live stack with:"
echo "  ros2 launch bringup bringup_nav3d.launch.py \\"
echo "    mode:=real use_localization:=true map:=$MAP_PATH"
echo
echo "Prereqs on the real machine (onboard OrinNX):"
echo "  - unitree_go2w driver package installed (onboard-only; absent from dev tree)"
echo "  - livox_ros_driver2 running (publishes /livox/lidar + /livox/imu)"
echo "  - Do NOT start livox_ros_driver2 when replaying a bag (use replay_lio.sh)"
echo "============================================================"

# Execute the full bringup. Root config selected by root_config_real.yaml
# (sensors/mid360_real.yaml + runtime/relocation_localization.yaml).
ros2 launch bringup bringup_nav3d.launch.py \
  mode:=real \
  use_localization:=true \
  map:="$MAP_PATH"
