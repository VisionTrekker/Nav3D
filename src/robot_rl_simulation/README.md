# robot_rl_simulation

A1 Gazebo Classic simulation package for the Nav3D RL execution layer.

## Current status

This package currently contains the ROS 2 package boundary, A1 ROS 2 control
description, body-state plugin, authorized policy and parking-stairs resources,
and pure safety-filter logic test. The runnable simulation launch remains
unavailable until `rl_locomotion_a1` and the Gazebo spawn/control launch graph
are implemented.

When the policy runtime is added, it will try CUDA first and fall back to CPU
only if CUDA is unavailable or initialization/inference fails. The active
backend must be logged explicitly. The runtime must still reject an invalid
model or an unexpected `[1, 225] -> 12` contract.
