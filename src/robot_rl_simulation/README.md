# robot_rl_simulation

A1 Gazebo Classic simulation package for the Nav3D RL execution layer.

## Current status

This package currently contains the ROS 2 package boundary and pure safety-filter
logic test. The TorchScript policy, Gazebo model plugin, ros2_control wiring, and
simulation launch are intentionally added only after their dependencies and
asset permissions are verified.

The policy runtime is required to use CUDA LibTorch and must fail fast when CUDA,
the model, or the expected `[1, 225] -> 12` contract is unavailable. It must not
fall back to CPU.
