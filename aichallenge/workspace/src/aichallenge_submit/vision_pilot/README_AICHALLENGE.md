# Vision Pilot integration

This package vendors the hybrid E2E perception pipeline from
`soyaoki/autoware_vision_pilot` at commit
`e93eceaaa70ee4e7b55fd4ecc04c023c163c8098` (Apache-2.0).

The upstream Apache-2.0 license and README are preserved in `LICENSE`,
`UPSTREAM_LICENSE`, and `UPSTREAM_README.md`. Attribution, the model-weight
license basis, and a summary of modifications are recorded in
`THIRD_PARTY_NOTICES.md`.

The following challenge-specific adaptations are applied:

- ROS image input is `/sensing/camera/image_raw`.
- INT8 AutoDrive, AutoSteer and AutoSpeed models are used with ONNX Runtime
  (CPU by default, or CUDA through the GPU Compose overlay).
- The upstream CppAD/IPOPT lateral MPC is replaced by a dependency-free
  curvature feed-forward plus CTE/heading feedback controller.
- `vision_pilot_adapter` converts the challenge `VelocityReport` and
  `AckermannControlCommand` APIs to/from Vision Pilot's scalar topics.
- Visualization and WebRTC are disabled for headless evaluation.

## Inference provider

Vision Pilot uses the ONNX Runtime CPU provider by default, so the normal
Compose configuration works without an NVIDIA GPU. The GPU overlay selects
CUDA automatically:

```bash
docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d autoware
```

Override either default explicitly when needed:

```bash
VISION_PILOT_PROVIDER=cpu docker compose up -d autoware
VISION_PILOT_PROVIDER=cuda docker compose \
  -f docker-compose.yml -f docker-compose.gpu.yml up -d autoware
```

The selected provider is printed at startup as `[OnnxEngine] provider=...`.
This setting changes Vision Pilot inference only; it does not select the AWSIM
camera readback mode or Unity renderer.

## Model weights

The three INT8 ONNX files are stored in this repository so a normal checkout
can be built and packaged without downloading anything. If a model is missing,
corrupted, or needs to be restored, download the exact files from the pinned
upstream commit and verify their SHA-256 checksums with:

```bash
make vision-pilot-models
```

The command is idempotent: already verified files are not downloaded again.
The files are placed under `modules/models/weights/`. Run it again if the local
model files are deleted or fail checksum verification.

The model files must exist before `make autoware-build`, creating a submission
archive, or building the evaluation image. No network access is required at
evaluation runtime.

Select it with `control_method:=vision_pilot`. Camera homography in
`config/H.yaml`, wheelbase, speed limit and controller gains require tuning for
the AI Challenge AWSIM camera and racing kart before competitive driving.
