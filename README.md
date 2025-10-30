# Grizzly Rover 2025-26

A ROS 2 robotics software stack for the WashU Robotics Rover Team's Grizzly rover platform. This repository contains a unified software architecture integrating core system management, perception, planning, control, and telemetry systems.

## Overview

The Grizzly stack is designed as a modular yet integrated system for autonomous rover operations. It follows a hierarchical architecture that manages multiple subsystems including drive, arm, and science operations.

### Key Features

- **Unified Architecture**: Single-package design (`grizzly_stack`) containing all major subsystems
- **Custom Interfaces**: Dedicated message definitions in `grizzly_interfaces`
- **Modular Components**: Separate nodes for perception, planning, and control
- **ROS 2 Native**: Built for ROS 2 Humble or later

## Architecture

### Package Structure

```
grizzly-25-26/
├── grizzly_interfaces/          # Custom ROS 2 message definitions
│   └── msg/
│       └── PerceptionState.msg  # Pose, twist, and terrain data
├── grizzly_stack/               # Main software stack
│   ├── core/                    # System management
│   │   └── system_manager.py   # Health monitoring and coordination
│   ├── perception/              # Sensor processing and state estimation
│   │   └── central_perception   # Central perception node
│   ├── planning/                # High-level planning nodes
│   │   ├── drive_planner
│   │   ├── arm_planner
│   │   ├── science_planner
│   │   └── fusion_planner
│   ├── control/                 # Low-level control nodes
│   │   ├── drive_controller
│   │   ├── arm_controller
│   │   └── science_controller
│   ├── telemetry/               # Data logging and monitoring
│   │   └── telemetry_unit
│   ├── launch/                  # Launch configurations
│   └── config/                  # YAML configuration files
└── build.sh                     # Build script with Python environment setup
```

### System Components

#### Core
- **System Manager**: Monitors system health and publishes status updates

#### Perception
- **Central Perception**: Processes sensor data and publishes `PerceptionState` messages including pose, velocity, and terrain classification

#### Planning
- **Drive Planner**: Path planning for rover navigation
- **Arm Planner**: Motion planning for robotic arm
- **Science Planner**: Task planning for science operations
- **Fusion Planner**: Coordinates multi-system operations

#### Control
- **Drive Controller**: Low-level control for rover locomotion
- **Arm Controller**: Joint-level control for manipulator
- **Science Controller**: Actuator control for science instruments

#### Telemetry
- **Telemetry Unit**: Logs and transmits rover data for monitoring

## Prerequisites

- **ROS 2**: Humble or later
- **Python**: 3.11 (configured via Anaconda)
- **Build Tools**: `colcon`, `ament_cmake`, `ament_python`
- **Anaconda Environment**: `ros_env` (as configured in `build.sh`)

### Dependencies

- `rclpy`
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`
- `launch` / `launch_ros`

## Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/WashU-Robotics-Rover/grizzly-25-26.git
   cd grizzly-25-26
   ```

2. **Set up ROS 2 environment**:
   ```bash
   source /opt/anaconda3/envs/ros_env/setup.zsh
   ```

3. **Build the workspace**:
   ```bash
   ./build.sh
   ```
   
   The build script automatically configures Python paths for the Anaconda `ros_env` environment.

4. **Source the workspace**:
   ```bash
   source install/setup.zsh
   ```

## Usage

### Launch Minimal System

To start the core system manager:

```bash
ros2 launch grizzly_stack grizzly_minimal.launch.py
```

### Run Individual Nodes

Launch specific components as needed:

```bash
# Core
ros2 run grizzly_stack system_manager

# Perception
ros2 run grizzly_stack central_perception

# Planning
ros2 run grizzly_stack drive_planner
ros2 run grizzly_stack arm_planner
ros2 run grizzly_stack science_planner
ros2 run grizzly_stack fusion_planner

# Control
ros2 run grizzly_stack drive_controller
ros2 run grizzly_stack arm_controller
ros2 run grizzly_stack science_controller

# Telemetry
ros2 run grizzly_stack telemetry_unit
```

## Configuration

Configuration files are located in `grizzly_stack/config/`:

- `core.yaml`: System manager parameters (e.g., health check rate)

Modify these files to adjust node behavior without rebuilding.

## Custom Messages

### PerceptionState

Defined in `grizzly_interfaces/msg/PerceptionState.msg`:

```
std_msgs/Header header
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
uint8 terrain_class  # 0=unknown, 1=flat, 2=sand, 3=rocky
```

This message provides integrated state estimation including position, orientation, velocity, and terrain classification.

## Development

### Building for Development

Use the provided build script with additional colcon arguments:

```bash
./build.sh --symlink-install  # For faster iteration during development
```

### Running Tests

```bash
colcon test
colcon test-result --verbose
```

## Project Info

- **Team**: WashU Robotics Rover Team
- **License**: MIT
- **Version**: 0.1.0
- **Academic Year**: 2025-26

## Contributing

This is an internal project for the WashU Robotics Rover Team. For team members:

1. Create a feature branch from `main`
2. Implement your changes
3. Test thoroughly
4. Submit a pull request for review

## Troubleshooting

### Python Environment Issues

If you encounter Python-related build errors, ensure:
- The `ros_env` Anaconda environment is activated
- Python 3.11 is properly installed in the environment
- The paths in `build.sh` match your Anaconda installation

### Build Failures

```bash
# Clean build
rm -rf build/ install/ log/
./build.sh
```

## Contact

For questions or issues, contact the WashU Robotics Rover Team at `huindam@gmail.com`.

---

**Go Bears! 🐻**
