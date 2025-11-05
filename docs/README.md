# Grizzly Rover Documentation

<div align="center">

**A ROS 2 robotics software stack for autonomous rover operations**

[Getting Started](#getting-started) • [Architecture](#architecture) • [ROS Reference](ROS_REFERENCE.md) • [Contributing](DEVELOPMENT.md)

</div>

---

## Overview

The **Grizzly Rover 2025-26** is a comprehensive ROS 2 software stack designed for the WashU Robotics Rover Team's autonomous rover platform. The system provides a unified, lifecycle-managed architecture with modular subsystems for perception, planning, control, and system management.

### Key Features

- 🚀 **Unified CLI**: Single `grizzly.py` command for all operations (install, build, run, test)
- 🌐 **Cross-Platform**: Support for macOS (robostack/conda), Linux (native ROS 2), and Windows
- 🔄 **Lifecycle Management**: ROS 2 Lifecycle nodes for deterministic state management
- 📦 **Modular Architecture**: Layer-based node organization for scalable system growth
- 🧪 **Comprehensive Testing**: Automated test suite with coverage reporting
- ⚙️ **Custom Interfaces**: Dedicated ROS 2 message definitions for rover-specific data
- 🔧 **Automated Installer**: Downloads and builds latest releases from GitHub

### System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Grizzly Rover Stack                       │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │   Core       │  │  Perception  │  │  Planning    │     │
│  │  System      │→ │  System      │→ │  System      │     │
│  │  Manager     │  │              │  │              │     │
│  └──────────────┘  └──────────────┘  └──────────────┘     │
│         │                  │                  │             │
│         └──────────────────┼──────────────────┘             │
│                            ↓                                 │
│                   ┌──────────────┐                          │
│                   │   Control    │                          │
│                   │   System     │                          │
│                   └──────────────┘                          │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## Getting Started

### Quick Start

Get up and running in minutes:

```bash
# 1. Clone the repository
git clone https://github.com/WashU-Robotics-Rover/grizzly-25-26.git
cd grizzly-25-26

# 2. Install (recommended - downloads latest release)
./grizzly.py install

# OR build from source
./grizzly.py build

# 3. Launch the system
./grizzly.py run

# 4. Run tests
./grizzly.py test
```

### Prerequisites

#### macOS (robostack/conda)
```bash
conda create -n ros_env python=3.11
conda activate ros_env
conda install -c robostack ros-humble-desktop colcon-common-extensions
```

#### Linux (Ubuntu 22.04)
```bash
# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
```

#### Windows
Follow the [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html).

### Command Reference

#### Installation & Building
```bash
./grizzly.py install              # Download latest release and build
./grizzly.py install --force      # Force re-download
./grizzly.py build                # Build from source
./grizzly.py build --clean        # Clean build (removes build artifacts)
./grizzly.py build --release      # Build in Release mode
./grizzly.py build --symlink-install  # Use symlinks for faster iteration
```

#### Running & Testing
```bash
./grizzly.py run                  # Launch the system
./grizzly.py run --debug          # Launch with debug output
./grizzly.py test                 # Run all tests
./grizzly.py test --coverage      # Generate coverage report
./grizzly.py test --html          # Generate HTML coverage report
./grizzly.py test --specific test_name  # Run specific test
```

## Architecture

### Core Components

#### System Manager
The central orchestration node that manages the rover's operational state machine. It coordinates between operational states (STARTUP, STANDBY, AUTONOMOUS, MANUAL, EMERGENCY, ERROR, SHUTDOWN) and delegates lifecycle control to the Layer Manager.

**Key Features:**
- Operational state machine management
- Health monitoring and reporting
- State transition coordination
- Emergency handling

📖 [Learn more about State Management](STATE_MANAGEMENT_GUIDE.md)

#### Layer Manager
Manages lifecycle transitions for nodes organized into logical layers. This architecture prevents system bloat as more nodes are added by grouping related nodes (perception, planning, control) and managing them as units.

**Key Features:**
- Layer-based node organization
- Automatic activation/deactivation based on operational state
- Async lifecycle transitions
- Configuration-driven layer definitions

📖 [Learn more about Layer Management](STATE_MANAGEMENT_GUIDE.md#layer-management)

#### Lifecycle Manager
Orchestration node that manages the startup and lifecycle transitions of all lifecycle-managed nodes. It provides event-based, deterministic node initialization that adapts to system performance.

**Key Features:**
- Event-based startup orchestration
- Node availability detection
- State confirmation and waiting
- Clean startup sequencing

📖 [Learn more about Lifecycle Manager](LIFECYCLE_MANAGER.md)

### Subsystems

#### Perception System
Sensor data processing, environmental awareness, and state estimation for the rover. Processes sensor inputs and publishes perception state messages.

📖 [Perception System Documentation](PERCEPTION_SYSTEM.md)

#### Planning System
High-level decision making and path planning for autonomous operations. Plans actions for drive, arm, and science subsystems.

#### Control System
Low-level motor and actuator control. Executes commands from the planning system.

#### Custom Interfaces
The `grizzly_interfaces` package defines rover-specific message types:
- `OperationalState` - System operational state with timestamp
- `PerceptionState` - Integrated state estimation (pose, twist, terrain)
- `NodeStatus` - Individual node health reporting
- `ChangeState` - Service for state transitions

## Documentation Guide

### For New Users

1. **[Getting Started Guide](DEVELOPMENT.md#development-environment-setup)** - Set up your development environment
2. **[System Architecture](ARCHITECTURE.md)** - Understand the system design
3. **[State Management Guide](STATE_MANAGEMENT_GUIDE.md)** - Learn about state management
4. **[Testing Guide](TESTING.md)** - Run and write tests

### For Developers

1. **[Development Guide](DEVELOPMENT.md)** - Complete development workflow
2. **[ROS Reference](ROS_REFERENCE.md)** - Complete ROS 2 nodes, topics, services, and messages reference
3. **[State Machine Guide](STATE_MACHINE_GUIDE.md)** - Detailed state machine reference
4. **[Lifecycle Manager](LIFECYCLE_MANAGER.md)** - Lifecycle orchestration details
5. **[Test Details](TEST_DETAILS.md)** - Test implementation details

### For System Architects

1. **[State Management Guide](STATE_MANAGEMENT_GUIDE.md)** - Comprehensive state management
2. **[Architecture Overview](ARCHITECTURE.md)** - System architecture deep dive
3. **[Layer Configuration](ROS_REFERENCE.md#layer-configuration)** - Layer management system

## Usage Examples

### Launch the System
```bash
./grizzly.py run
```

### Monitor System State
```bash
# Monitor current operational state
ros2 topic echo /system/state

# Monitor health status
ros2 topic echo /system/health
```

### Change Operational State
```bash
# Transition to AUTONOMOUS mode
ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
  "{requested_state: 2, reason: 'Starting autonomous mission'}"

# Trigger emergency stop
ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
  "{requested_state: 4, reason: 'EMERGENCY STOP'}"

# Return to STANDBY
ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
  "{requested_state: 1, reason: 'Emergency cleared'}"
```

### Lifecycle Management
```bash
# Get current lifecycle state
ros2 lifecycle get /system_manager

# Manually control lifecycle
ros2 lifecycle set /system_manager configure
ros2 lifecycle set /system_manager activate
```

### View Available Nodes
```bash
ros2 node list
ros2 node info /system_manager
```

## Configuration

Configuration files are located in `grizzly_stack/config/`:

- **`core.yaml`** - System parameters (health rate, etc.)
- **`layers.yaml`** - Layer definitions and node organization

Modify these files and relaunch (no rebuild required).

## Troubleshooting

### Build Issues
```bash
# Clean build
./grizzly.py build --clean

# Check environment (macOS)
conda activate ros_env
which python

# Check environment (Linux)
source /opt/ros/humble/setup.bash
ros2 --version
```

### Runtime Issues
```bash
# Verify workspace is sourced
source install/setup.bash  # Linux
source install/setup.zsh   # macOS

# Check if nodes are running
ros2 node list

# Enable debug output
./grizzly.py run --debug
```

For detailed troubleshooting, see the [Development Guide](DEVELOPMENT.md#troubleshooting).

## Project Status

### ✅ Implemented
- Cross-platform CLI tool (`grizzly.py`)
- System manager with lifecycle management
- Layer-based node management system
- Operational state machine (7 states)
- Custom ROS 2 interfaces
- Automated testing (21+ tests)
- Configuration management
- Lifecycle orchestration
- Perception subsystem foundation

### 🚧 In Progress
- Enhanced perception processing
- Planning subsystem implementation
- Control subsystem integration
- Telemetry system

### 📋 Planned
- Navigation subsystem
- Science operations
- Teleoperation interface
- Advanced diagnostics

## Contributing

This is an internal project for the WashU Robotics Rover Team.

### Contribution Workflow

1. Create a feature branch: `git checkout -b feature/your-feature-name`
2. Make your changes and test: `./grizzly.py test`
3. Commit and push: `git push origin feature/your-feature-name`
4. Create a Pull Request on GitHub

See the [Development Guide](DEVELOPMENT.md) for detailed contribution guidelines.

## Project Information

| Item | Details |
|------|---------|
| **Team** | WashU Robotics Rover Team |
| **Repository** | https://github.com/WashU-Robotics-Rover/grizzly-25-26 |
| **Academic Year** | 2025-26 |
| **ROS 2 Distribution** | Humble |
| **License** | MIT |
| **Contact** | huindam@gmail.com |

---

<div align="center">

**Go Bears! 🐻**

Built with ❤️ by the WashU Robotics Rover Team

</div>
