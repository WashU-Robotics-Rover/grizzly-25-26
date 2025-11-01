#!/bin/zsh
#
# Build script for Grizzly workspace with proper Python configuration
#
# This script builds all ROS 2 packages in the workspace using colcon.
# It's specifically configured to work with a conda environment named 'ros_env'
# on macOS, where finding the correct Python libraries can be tricky.
#
# Usage:
#   ./build.sh                    # Build all packages
#   ./build.sh --packages-select grizzly_stack  # Build specific package
#   ./build.sh --symlink-install  # Use symlinks (faster iteration)
#
# Why explicit Python paths?
# - CMake sometimes finds the wrong Python installation on macOS
# - Conda environments require explicit paths to ensure consistency
# - This prevents mixing Python versions between build and runtime

# --- ACTIVATE CONDA ENVIRONMENT ---

# Initialize conda for shell script (required for conda activate to work in scripts)
eval "$(conda shell.zsh hook)"

# Activate the ros_env conda environment
conda activate ros_env

# --- SETUP ROS 2 ENVIRONMENT ---

# Source the ROS 2 environment from the conda environment
# This sets up all ROS 2 environment variables (ROS_DISTRO, AMENT_PREFIX_PATH, etc.)
source /opt/anaconda3/envs/ros_env/setup.zsh

# --- BUILD PACKAGES ---

# Use colcon to build all packages in the workspace
# colcon is the standard build tool for ROS 2 workspaces
# We explicitly specify Python paths to ensure CMake finds the correct Python installation
# from the conda environment (prevents mixing Python versions)
colcon build \
  --cmake-args \
    -DPython_EXECUTABLE=/opt/anaconda3/envs/ros_env/bin/python \
    -DPython_INCLUDE_DIR=/opt/anaconda3/envs/ros_env/include/python3.11 \
    -DPython_LIBRARY=/opt/anaconda3/envs/ros_env/lib/libpython3.11.dylib \
  "$@"

# --- SOURCE THE WORKSPACE ---

# Source the newly built workspace so we can use the packages immediately
# This adds the workspace packages to ROS_PACKAGE_PATH and PYTHONPATH
source install/setup.zsh

echo ""
echo "Build complete! Workspace is now sourced and ready to use."
echo "Run 'ros2 launch grizzly_stack grizzly_minimal.launch.py' to start the system."
