cmake_minimum_required(VERSION 3.22.1)

project(vision_pilot VERSION 1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_definitions(VISIONPILOT_SHARE_DIR="${CMAKE_INSTALL_PREFIX}/share/vision_pilot")

# Put all executables in build/
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Testing (default OFF; users can override with -DBUILD_TESTING=ON)
set(BUILD_TESTING OFF CACHE BOOL "Build the testing tree")

# GPU support
option(GPU "Enable GPU support" OFF)

# ROS2 packages
option(ENABLE_ROS2_INTERFACE "Enable ROS2 support" ON)

# Build testing code
option(BUILD_TESTING "Build the testing tree" OFF)

# Rerun C++ SDK
# Disabled by default
# Resolved either from a local checkout pointed to by RERUN_SDK_ROOT (env or cache var),
# or downloaded via FetchContent.
option(ENABLE_RERUN "Log directly to a Rerun .rrd via the Rerun C++ SDK" OFF)

set(Python3_EXECUTABLE /usr/bin/python3 CACHE FILEPATH "" FORCE)
find_package(Python3 REQUIRED)

add_custom_target(generate_config ALL
        COMMAND Python3::Interpreter
        ${CMAKE_SOURCE_DIR}/scripts/find_homography_C_matrix.py
        --ground-h ${CMAKE_SOURCE_DIR}/config/H.yaml
        --output ${CMAKE_BINARY_DIR}/config/homography_C_matrix.yaml
)

# Modules
add_subdirectory(modules/logging)

add_subdirectory(modules/config)

add_subdirectory(modules/common)

add_subdirectory(modules/sensing/image_preprocessing)

if(ENABLE_ROS2_INTERFACE)
    add_subdirectory(modules/middleware_interfaces/ros2_interface/camera_ros2_interface)
    add_subdirectory(modules/middleware_interfaces/ros2_interface/vehicle_ros2_interface)
endif()

add_subdirectory(modules/engine)
add_subdirectory(modules/models)

add_subdirectory(modules/safety_guardian/fusion)   # tracking is embedded inside fusion
add_subdirectory(modules/safety_guardian/planning)
add_subdirectory(modules/debug)

add_subdirectory(modules/sensing/vehicle_interface)
add_subdirectory(modules/sensing/camera_interface)

# config
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# App
add_subdirectory(app)

# Make the upstream standalone CMake project discoverable by colcon/ament and
# install it into the overlay prefix used by the challenge runtime.
ament_package()
