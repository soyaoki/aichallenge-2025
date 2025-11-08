# VLM Planner

VLM-based trajectory planner for autonomous driving.

## Overview

The VLM Planner generates trajectory using VLM inference based on camera images and vehicle state. Unlike trajectory selectors that choose from candidates, this planner creates entirely new trajectory paths.

## Setup

### Environment Setup

```sh
uv venv -p python3.10
```

```sh
source .venv/bin/activate
```

```sh
uv pip install .
```

### Set Gemini API Key

```sh
export GEMINI_API_KEY="YOUR_API_KEY"
```

## Usage

**Files:**
- `vlm_planner_node.py` - Main ROS 2 node
- `vlm_planner.py` - VLM trajectory generation logic
- `prompt.py` - Trajectory generation prompts and utilities

**Run:**
```sh
# Run the VLM planner node with custom output topic
python vlm_planner_node.py --ros-args -p output_topic:="/planning/ml_planner/auto/trajectory"
```

**Topics:**
- Subscribe:
  - `/sensing/camera/image_raw` (Image)
  - `/localization/kinematic_state` (Odometry)
  - `/localization/acceleration` (AccelWithCovarianceStamped)
- Publish:
  - `/output/trajectory` (Trajectory)

**Parameters:**
- `output_topic`: Output trajectory topic (default: `/output/trajectory`)
- Inference interval: 5.0 seconds (configurable in code)

## Track Knowledge

This component includes knowledge of a 13-sector race track:

1. Starting Straight
2. R-Hairpin (with white sign landmark)
3. Short Straight
4. L-Hairpin
5. Short Straight
6. U-shaped Right
7. 90-degree Left
8. R-Hairpin
9. L-Hairpin
10. S-Curve (L->R)
11. 90-degree Right
12. S-Curve (L->R)
13. Sweeping Right Corner

## Architecture

```
Camera Image + Vehicle State → VLM (Gemini) → Trajectory Generation → ROS 2 Message → Autoware
```

The VLM Planner uses Google's Gemini 2.5 Flash Lite model for fast inference while maintaining accuracy for autonomous driving trajectory generation.

## Development

- `VLMPlanner` class handles trajectory generation using Gemini
- `VlmPlannerNode` handles ROS 2 communication and vehicle state management
- Trajectory points include 3D coordinates, timing, and velocity information

## Dependencies

- ROS 2

## Acknowledgment

The implementation of this code was greatly inspired by the following repository. Many thanks for their excellent work:
https://github.com/soyaoki/AWSIM-VLM-Drive
