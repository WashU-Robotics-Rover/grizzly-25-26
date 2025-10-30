#!/bin/zsh
# Build script for grizzly workspace with proper Python configuration

# Source ROS environment
source /opt/anaconda3/envs/ros_env/setup.zsh

# Build with Python paths explicitly set
colcon build \
  --cmake-args \
    -DPython_EXECUTABLE=/opt/anaconda3/envs/ros_env/bin/python \
    -DPython_INCLUDE_DIR=/opt/anaconda3/envs/ros_env/include/python3.11 \
    -DPython_LIBRARY=/opt/anaconda3/envs/ros_env/lib/libpython3.11.dylib \
  "$@"
