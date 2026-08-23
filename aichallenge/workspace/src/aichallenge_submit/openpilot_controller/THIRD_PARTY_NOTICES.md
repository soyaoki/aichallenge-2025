# Third-party notices

## openpilot (comma.ai)

This package contains code and a neural network weight file derived from
[commaai/openpilot](https://github.com/commaai/openpilot), pinned at commit
`084747c75d2cbd23af65ab7a9e770bbd7b98bac9`.

openpilot is distributed under the MIT License, reproduced in
[`UPSTREAM_LICENSE`](UPSTREAM_LICENSE). The same license covers the model
weights, which live in the openpilot repository under Git LFS.

### Files ported from openpilot

| This package | Upstream | Modifications |
|---|---|---|
| `openpilot_controller/model_constants.py` | `openpilot/selfdrive/modeld/constants.py` | Reformatted only. |
| `openpilot_controller/parse_model_outputs.py` | `openpilot/selfdrive/modeld/parse_model_outputs.py` | Reformatted; import path changed. |
| `openpilot_controller/drive_helpers.py` | `openpilot/selfdrive/controls/lib/drive_helpers.py` | Reduced to the plan-to-action helpers; `clip_curvature` limits and time steps made arguments. |
| `openpilot_controller/transforms.py` | `common/transformations/{camera,model,orientation}.py`, `openpilot/selfdrive/modeld/compile_modeld.py` | Model intrinsics and `get_warp_matrix` ported as-is; the tinygrad GPU warp and `frames_to_tensor` reimplemented with OpenCV/NumPy. |
| `openpilot_controller/supercombo.py` | `openpilot/selfdrive/modeld/modeld.py`, `compile_modeld.py` | Input-queue semantics ported; the tinygrad runner replaced with ONNX Runtime. |

### Model weights

`models/driving_supercombo.onnx` is downloaded verbatim from the pinned commit
by `scripts/download_models.bash`. It is not redistributed in this repository.

## Modifications for the AI Challenge

- Input comes from a single ROS camera instead of openpilot's road/wide camera
  pair; the same frame feeds both `img` and `big_img`.
- Camera intrinsics come from `sensor_msgs/CameraInfo` (or a horizontal FOV
  parameter) instead of openpilot's hardware camera table, and the calibration
  euler angles are parameters instead of `liveCalibration`.
- The desire input is always zero: there is no lane-change planner.
- Actions are converted to `autoware_auto_control_msgs/AckermannControlCommand`
  instead of being published as `modelV2`.
