# source /autoware/install/setup.bash 
# source /opt/ros/humble/setup.bash

# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
uv run python3 -m trajectory_selector --ros-args -p input_topic:="/planning/vad/trajectories_base" -p output_topic:="/planning/ml_planner/auto/trajectory"
