#!/usr/bin/env bash
# anomaly_lost.sh - Test LOST fallback after injecting a wrong /initialpose
# Part of: test(anomaly): STOP + LOST fallback paths (#13)
#
# LIMITATION: upstream lio (lio_sam) does NOT subscribe to /initialpose
# (verified: only match in tree is an rviz topic entry in
# src/lio/src/sim/sim.rviz, not a node subscription). /initialpose is the
# standard RViz 2D Pose Estimate topic. So this test validates that the
# topic can be injected into the system, but lio will NOT react to it -- the
# LOST->recover fallback path is not actually exercised end-to-end here.
set -euo pipefail

# Launch bringup in bag mode with localization
ros2 launch bringup bringup_nav3d.launch.py mode:=bag use_localization:=true &
LAUNCH_PID=$!
trap 'kill $LAUNCH_PID 2>/dev/null || true' EXIT

# Allow system to stabilize
sleep 15

# Inject wrong initial pose far from true location (100m offset) --
# in upstream lio this is not consumed, but the injection itself is exercised.
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped \
  '{header: {frame_id: "map"}, pose: {pose: {position: {x: 100, y: 100, z: 0}, orientation: {w: 1.0}}}}'

# Give any fallback/re-localization time to act
sleep 5

# Upstream lio publishes no /score topic (adjudicated Task 5) -- use the
# odom publish rate as the observable metric that reflects localization
# health instead. If odom keeps publishing at a healthy rate, the pipeline is
# alive; a drop would indicate localization degradation.
timeout 10 ros2 topic hz /lio/localization/odom --window 5
