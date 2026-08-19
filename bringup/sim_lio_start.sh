#!/usr/bin/env bash
# Start Nav3D LIO (Elevator-LIO) with the GazeboQuadbot mid360 simulation.
#
# Prereq: GazeboQuadbot sim already running, publishing:
#   /livox/lidar  (livox_ros_driver2/msg/CustomMsg)   <-- direct, no bridge
#   /livox/imu    (sensor_msgs/msg/Imu, ~111 Hz)
#
# Uses the sim root_config_sim.yaml (mid360 <-> dog body mount via
# lidar_t_body / lidar_R_body, elevator mode disabled).
#
# Usage:
#   bash bringup/sim_lio_start.sh                # lio + rviz2
#   bash bringup/sim_lio_start.sh --no-rviz      # lio only
#
# Run in a clean terminal to avoid polluted ROS env (the .bash shebang is
# intentional -- BASH_SOURCE[0] is empty in zsh, so sourcing setup.bash from
# a zsh shell with a CWD inside a package dir produces wrong prefix detection).
set -e

NAV3D_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# --- Clean environment (avoid stale ROS workspace pollution) ---
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH 2>/dev/null || true
export ROS_LOCALHOST_ONLY=0

# --- Source ROS + GazeboQuadbot + Nav3D (bash variants for BASH_SOURCE) ---
source /opt/ros/humble/setup.bash
if [ -f /media/lenovo/disk/Embodied_AI/GazeboQuadbot/install/setup.bash ]; then
  source /media/lenovo/disk/Embodied_AI/GazeboQuadbot/install/setup.bash
fi
source "$NAV3D_ROOT/install/setup.bash"

LIO_CONFIG="root_config_sim.yaml"   # resolved by Elevator-LIO under pkg yaml/
LIO_PKG="lio"
LIO_LAUNCH="start_ros2.launch.py"

# Sanity: confirm Elevator-LIO and sim configs are present
if ! ros2 pkg prefix "$LIO_PKG" >/dev/null 2>&1; then
  echo "ERROR: ros2 package '$LIO_PKG' not found. Build first:" >&2
  echo "  cd $NAV3D_ROOT && colcon build --packages-select lio --cmake-args -DHUMBLE_ROS=humble" >&2
  exit 1
fi
if [ ! -f "$NAV3D_ROOT/src/Elevator-LIO/yaml/$LIO_CONFIG" ]; then
  echo "ERROR: $LIO_CONFIG not found under src/Elevator-LIO/yaml/" >&2
  exit 1
fi

echo "[1/2] Starting Elevator-LIO (mapping)"
echo "      config : $LIO_CONFIG"
echo "      lidar  : /livox/lidar   (CustomMsg from livox_plugin)"
echo "      imu    : /livox/imu     (sensor_msgs/Imu, ~111 Hz)"
echo "      body-xf: lidar_t_body=[-0.2809, 0, -0.0224]  lidar_R_body=R_y(0.4)"
ros2 launch "$LIO_PKG" "$LIO_LAUNCH" "config_path:=$LIO_CONFIG" "use_rviz:=false" &
LIO_PID=$!

LAUNCH_RVIZ=1
for arg in "$@"; do
  case "$arg" in
    --no-rviz) LAUNCH_RVIZ=0 ;;
  esac
done

if [ "$LAUNCH_RVIZ" = "1" ]; then
  echo "[2/2] Starting RViz2"
  RVIZ_CFG="$NAV3D_ROOT/install/lio/share/lio/rviz/LIO_ros2.rviz"
  if [ -f "$RVIZ_CFG" ]; then
    rviz2 -d "$RVIZ_CFG" &
    RVIZ_PID=$!
  else
    echo "      (rviz config not found, skipping)"
    RVIZ_PID=
  fi
else
  RVIZ_PID=
fi

echo
echo "lio (Elevator-LIO) running."
echo "  TF    : world -> IMU  (use ros2 run tf2_ros tf2_echo world IMU)"
echo "  Topics: /LIO/{odom_imu, odom_vehicle, clouds_lidar, global_map, ikdtree, in_elevator}"
echo "  Drive the dog with teleop_twist_keyboard to populate the map."
echo "  Ctrl-C to stop both."

cleanup() {
  echo "Stopping..."
  [ -n "$LIO_PID" ] && kill "$LIO_PID" 2>/dev/null || true
  [ -n "$RVIZ_PID" ] && kill "$RVIZ_PID" 2>/dev/null || true
}
trap cleanup INT TERM
wait