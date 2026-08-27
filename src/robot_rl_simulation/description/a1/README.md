# A1 ROS 2 control profile

This directory contains the BSD-3-Clause Unitree A1 morphology adapted for ROS 2
Humble and Gazebo Classic. The original ROS 1 control plugins, truth odometry,
foot-contact plugins, Livox mesh, and sensor plugins are intentionally excluded.

`robot.xacro` exposes exactly twelve effort-commanded joints. Their configured
controller order is the policy order:

```text
FR_hip_joint FR_thigh_joint FR_calf_joint
FL_hip_joint FL_thigh_joint FL_calf_joint
RR_hip_joint RR_thigh_joint RR_calf_joint
RL_hip_joint RL_thigh_joint RL_calf_joint
```

Each joint has an URDF effort limit of 33.5 N·m. The eventual RL adapter must
parse and enforce those URDF limits; `controllers.yaml` only forwards effort.
The Gazebo launch remains unavailable until `rl_locomotion_a1` and the
Gazebo spawn/control launch graph are implemented.
