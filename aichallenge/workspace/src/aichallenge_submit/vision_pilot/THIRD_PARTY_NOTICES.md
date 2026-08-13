# Third-party notices

## Autoware Vision Pilot

This directory contains a modified copy of Autoware Vision Pilot obtained
from:

- Project: Autoware Vision Pilot
- Upstream: https://github.com/autowarefoundation/autoware_vision_pilot
- Integration source: https://github.com/soyaoki/autoware_vision_pilot
- Pinned revision: `e93eceaaa70ee4e7b55fd4ecc04c023c163c8098`
- License: Apache License 2.0

The upstream project describes its complete codebase, including the
AutoDrive, AutoSteer, and AutoSpeed model weights, as released under the
Apache License 2.0. The license text distributed with the upstream project is
preserved in `LICENSE` and `UPSTREAM_LICENSE`; the original project README is
preserved in `UPSTREAM_README.md`.

This copy has been modified for the Automotive AI Challenge racing-kart
environment. Material integration changes include:

- ROS 2 camera input and challenge-specific topic adaptation;
- ONNX Runtime CPU/CUDA provider selection;
- INT8 model selection and reproducible model download/checksum handling;
- replacement of the upstream CppAD/IPOPT lateral MPC with a compact
  curvature feed-forward plus CTE/heading feedback controller;
- challenge-specific launch, configuration, headless operation, and debug
  output changes.

The modifications are not endorsed by or maintained by the Autoware
Foundation. See `README_AICHALLENGE.md` and the repository history for the
implementation details and exact changes.

