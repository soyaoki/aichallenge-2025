cmake_minimum_required(VERSION 3.8)
project(openpilot_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install entry point script for the node
install(PROGRAMS
  openpilot_controller/openpilot_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/download_models.bash
  DESTINATION lib/${PROJECT_NAME}
)

# The ONNX weights are downloaded by `make openpilot-models`, not committed.
install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
