# Third-party notices

## A1 description and meshes

- Source: `https://github.com/Robot-Nav/legbot_3D_Nav`, commit
  `f60606a4903bad58fca813f76072f9940a284d1d`.
- Embedded Unitree description code is covered by the Unitree BSD-3-Clause
  license, not the repository's Apache-2.0 license.
- The BSD-3-Clause copyright, conditions, and disclaimer must remain with any
  copied source or binary distribution. No endorsement is implied.
- This package will preserve the upstream license in the copied description
  directory before distributing those assets.

## Policy weights

`policy_act_inference_stair.pt` is copied from the authorized reference source:

- Source repository: `https://github.com/Robot-Nav/legbot_3D_Nav`
- Source commit: `b6c48fd9141e2e6d456d0890a5c139206c1d534c`
- SHA-256: `2d5aa72511c0c6609c02f4105845eee6974d3d73431497f8f35306da9588fe14`
- Target: `policies/a1/policy_act_inference_stair.pt`
- Redistribution status: authorized by the project owner for this Nav3D workspace.

## Parking-stairs asset

The parking-stairs geometry is copied from the authorized local reference asset:

- Source: authorized local reference asset `1_Building/Building.dae`
- SHA-256: `2b297b58fd4352d87b005b5ce1e0ce98119083be6805b1b25c4478f075c63bd9`
- Target: `models/parking_stairs/meshes/parking_stairs.dae`
- Redistribution status: authorized by the project owner for this Nav3D workspace.

The model and world paths are normalized to package-relative Gazebo `model://`
URIs; the original external absolute paths are not used at runtime.