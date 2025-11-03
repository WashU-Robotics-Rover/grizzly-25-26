# CMakeLists.txt for grizzly_interfaces
#
# This CMake configuration file defines how to build the grizzly_interfaces package,
# which contains custom ROS 2 message definitions for the Grizzly rover.
#
# Key tasks:
# 1. Find required dependencies (ROS 2 packages)
# 2. Generate code for custom messages (.msg files)
# 3. Export dependencies for packages that depend on this one

# Require CMake 3.8 or higher (standard for ROS 2)
cmake_minimum_required(VERSION 3.8)

# Define the project name (must match package.xml)
project(grizzly_interfaces)

# --- FIND REQUIRED PACKAGES ---

# ament_cmake: The build system for ROS 2 CMake packages
find_package(ament_cmake REQUIRED)

# rosidl_default_generators: Tools to generate code from .msg, .srv, .action files
# This creates C++, Python, and other language bindings for our custom messages
find_package(rosidl_default_generators REQUIRED)

# Message type dependencies: packages that define message types we use
find_package(std_msgs REQUIRED)       # Standard message types (Header, String, etc.)
find_package(geometry_msgs REQUIRED)  # Geometric types (Pose, Twist, etc.)
find_package(nav_msgs REQUIRED)       # Navigation types (Odometry, Path, etc.)

# --- GENERATE CUSTOM INTERFACE CODE ---

# Generate code for all custom message definitions
# This creates libraries and Python modules that can be imported/used in code
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PerceptionState.msg"  # List all .msg files here
  DEPENDENCIES std_msgs geometry_msgs nav_msgs  # Types used within our messages
)

# --- EXPORT CONFIGURATION ---

# Export the runtime dependency so dependent packages know to link against rosidl
ament_export_dependencies(rosidl_default_runtime)

# Finalize the ament package configuration
ament_package()
