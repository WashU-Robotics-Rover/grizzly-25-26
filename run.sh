#!/bin/zsh
#
# Run script for Grizzly workspace
#
# This script sources the workspace environment and launches the minimal Grizzly system.
# It's a convenient shortcut to avoid typing the full launch command each time.
#
# Usage:
#   ./run.sh                    # Launch the minimal system
#
# Prerequisites:
#   - Workspace must be built first (run ./build.sh)
#   - ROS 2 environment must be available (conda env: ros_env)
#
# What this script does:
# 1. Activates the conda environment (ros_env)
# 2. Sources the ROS 2 base environment from conda
# 3. Sources the local workspace overlay (install/setup.zsh)
# 4. Launches the grizzly_minimal.launch.py file

# --- ACTIVATE CONDA ENVIRONMENT ---

# Initialize conda for shell script (required for conda activate to work in scripts)
eval "$(conda shell.zsh hook)"

# Activate the ros_env conda environment
conda activate ros_env

# --- SOURCE ROS 2 BASE ENVIRONMENT ---

# Source the ROS 2 environment from the conda environment
# This sets up all ROS 2 environment variables
source /opt/anaconda3/envs/ros_env/setup.zsh

# --- SOURCE LOCAL WORKSPACE ---

# Source the local workspace overlay
# This adds our custom packages to the ROS 2 environment
# The install directory is created by 'colcon build'
if [ -f "install/setup.zsh" ]; then
    source install/setup.zsh
else
    echo "Error: Workspace not built. Please run ./build.sh first."
    exit 1
fi

# --- LAUNCH THE SYSTEM ---

echo ""
echo "================================================"
echo "  Launching Grizzly Minimal System"
echo "================================================"
echo ""
echo "Starting nodes:"
echo "  - system_manager (lifecycle node)"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo ""

# Launch the minimal system
# This starts the system_manager node and automatically transitions it to Active state
ros2 launch grizzly_stack grizzly_minimal.launch.py
