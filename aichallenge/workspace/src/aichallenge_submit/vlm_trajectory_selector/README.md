# VLM Trajectory Selector

This package provides VLM-based trajectory selection functionality for autonomous driving.

## Overview

The VLM Trajectory Selector receives multiple candidate trajectories and uses a Vision Language Model to select the most appropriate one based on the current visual input from the vehicle's camera.

## Features

- Vision-based trajectory selection from candidates
- Integration with Autoware trajectory messages
- Real-time decision making

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

## Components

### 1. VLM Trajectory Selector

Selects the best trajectory from a set of candidate trajectories based on camera images and VLM inference.

**Files:**
- `trajectory_selector.py` - Main ROS 2 node
- `vlm_selector.py` - VLM inference logic

**Run:**
```sh
# Run the trajectory selector node with custom topics
python -m trajectory_selector --ros-args -p input_topic:="/planning/vad/trajectories_base" -p output_topic:="/planning/ml_planner/auto/trajectory"
```

**Topics:**
- Subscribe: 
  - `/input/trajectory` (CandidateTrajectories)
  - `/sensing/camera/image_raw` (Image)
- Publish: 
  - `/output/trajectory` (Trajectory)

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
Camera Image → VLM (Gemini) → Trajectory Selection → ROS 2 Message → Autoware
```

The VLM Trajectory Selector uses Google's Gemini 2.5 Flash Lite model for fast inference while maintaining accuracy for autonomous driving decisions.

## Development

- `VLMSelector` class handles Gemini API calls and image preprocessing
- `VlmTrajectorySelectorNode` handles ROS 2 communication and message conversion


## Acknowledgment

The implementation of this code was greatly inspired by the following repository. Many thanks for their excellent work:
https://github.com/soyaoki/AWSIM-VLM-Drive
