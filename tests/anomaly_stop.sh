#!/usr/bin/env bash
# anomaly_stop.sh - Test STOP protocol: /local_planner/cmd_vel zeroes out when bag stops
# Part of: test(anomaly): STOP + LOST fallback paths (#13)
#
# Scenario: sensor/data anomaly -> bringup should STOP the robot (zero cmd_vel)
# within a short timeout instead of continuing to command motion.
set -euo pipefail

# Launch bringup in bag mode with localization
ros2 launch bringup bringup_nav3d.launch.py mode:=bag use_localization:=true &
LAUNCH_PID=$!
trap 'kill $LAUNCH_PID 2>/dev/null || true' EXIT

# Allow system to stabilize
sleep 15

# Kill bag playback -- simulates a data-feed anomaly cutting input
pkill -9 -f "ros2 bag play"

# Give the STOP protocol time to detect the loss and zero cmd_vel
sleep 1

# STOP assertion: after killing the bag feed, cmd_vel must go zero (or the topic go silent).
# Humble's `ros2 topic hz` never self-exits (spins forever), so it cannot
# distinguish a stopped topic from a still-publishing one -- timeout just kills
# it at the limit in both cases. Instead, sample the message value directly.
STOP_OK=0
if CMD_VEL=$(timeout 2 ros2 topic echo /local_planner/cmd_vel geometry_msgs/msg/Twist --once 2>/dev/null); then
  # A message arrived within 2s -- verify every linear AND angular velocity
  # component is ~zero (Twist echo prints `linear: {x,y,z}` and `angular: {x,y,z}`,
  # with the value in the second whitespace-separated field, e.g. `  x: 0.5`).
  # Any component with magnitude above a tiny epsilon => STOP protocol did not trip.
  if printf '%s\n' "$CMD_VEL" | awk '
      /^[ ]*(linear|angular):/ { in_sec=1; next }
      in_sec && /[xyz]:/ {
        v=$2
        gsub(/[^0-9.\-]/, "", v)
        if (v+0 < -0.0001 || v+0 > 0.0001) nz=1
      }
      END { exit (nz ? 1 : 0) }
    '; then
    STOP_OK=1
  fi
else
  # No message arrived (publisher stopped entirely) -- also acceptable
  STOP_OK=1
fi
if [ "$STOP_OK" -ne 1 ]; then
  echo "FAIL: cmd_vel did not stop (nonzero velocity) after bag kill" >&2
  exit 1
fi
echo "PASS: cmd_vel zero/silent after bag kill"
