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

# --- AUTO-DETECT CONDA ENVIRONMENT (macOS) ---
# Automatically detect and use conda Python paths when CONDA_PREFIX is set
# This prevents build failures when switching between conda environments
# Note: Set these BEFORE find_package(Python) to override any cached values
if(APPLE AND DEFINED ENV{CONDA_PREFIX})
  set(CONDA_PREFIX $ENV{CONDA_PREFIX})
  
  # Find Python executable in conda environment
  find_program(CONDA_PYTHON_EXECUTABLE
    NAMES python3 python
    PATHS ${CONDA_PREFIX}/bin
    NO_DEFAULT_PATH
  )
  
  if(CONDA_PYTHON_EXECUTABLE)
    # Get Python version
    execute_process(
      COMMAND ${CONDA_PYTHON_EXECUTABLE} --version
      OUTPUT_VARIABLE PYTHON_VERSION_OUTPUT
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    string(REGEX REPLACE "Python ([0-9]+\\.[0-9]+).*" "\\1" PYTHON_VERSION ${PYTHON_VERSION_OUTPUT})
    
    # Set Python paths explicitly (FORCE to override cached values)
    set(Python_EXECUTABLE ${CONDA_PREFIX}/bin/python CACHE FILEPATH "Python executable" FORCE)
    set(Python3_EXECUTABLE ${CONDA_PREFIX}/bin/python3 CACHE FILEPATH "Python3 executable" FORCE)
    set(Python_INCLUDE_DIR ${CONDA_PREFIX}/include/python${PYTHON_VERSION} CACHE PATH "Python include directory" FORCE)
    set(Python_LIBRARY ${CONDA_PREFIX}/lib/libpython${PYTHON_VERSION}.dylib CACHE FILEPATH "Python library" FORCE)
    
    # Verify paths exist
    if(EXISTS ${Python_EXECUTABLE} AND EXISTS ${Python_INCLUDE_DIR} AND EXISTS ${Python_LIBRARY})
      message(STATUS "Using conda Python: ${Python_EXECUTABLE}")
      message(STATUS "  Include: ${Python_INCLUDE_DIR}")
      message(STATUS "  Library: ${Python_LIBRARY}")
    else()
      # Reset if paths don't exist
      unset(Python_EXECUTABLE CACHE)
      unset(Python3_EXECUTABLE CACHE)
      unset(Python_INCLUDE_DIR CACHE)
      unset(Python_LIBRARY CACHE)
    endif()
  endif()
endif()

# --- FIND REQUIRED PACKAGES ---

# ament_cmake: The build system for ROS 2 CMake packages
find_package(ament_cmake REQUIRED)

# rosidl_default_generators: Tools to generate code from .msg, .srv, .action files
# This creates C++, Python, and other language bindings for our custom messages
# IMPORTANT: Find this BEFORE other message dependencies to ensure proper typesupport setup
find_package(rosidl_default_generators REQUIRED)

# Message type dependencies: packages that define message types we use
# Order matters: builtin_interfaces should come first as it's a base dependency
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)       # Standard message types (Header, String, etc.)
find_package(geometry_msgs REQUIRED)  # Geometric types (Pose, Twist, etc.)
find_package(nav_msgs REQUIRED)       # Navigation types (Odometry, Path, etc.)

# --- GENERATE CUSTOM INTERFACE CODE ---

# Generate code for all custom message definitions
# This creates libraries and Python modules that can be imported/used in code
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NodeStatus.msg"         # Node health status message
  "msg/PerceptionState.msg"    # Perception subsystem state
  "msg/OperationalState.msg"   # Global operational state machine
  "srv/ChangeState.srv"        # Service to change operational state
  DEPENDENCIES std_msgs geometry_msgs nav_msgs builtin_interfaces  # Types used within our messages
)

# --- EXPORT CONFIGURATION ---

# Export dependencies for packages that depend on this one
# This ensures proper linking and typesupport resolution across platforms
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(builtin_interfaces)

# Finalize the ament package configuration
ament_package()
