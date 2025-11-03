# Grizzly Rover 2025-26

A ROS 2 robotics software stack for the WashU Robotics Rover Team's Grizzly rover platform. This repository provides a unified software architecture with automated release management and cross-platform build support for macOS, Linux, and Windows.

## Overview

The Grizzly stack is designed as a modular, lifecycle-managed system for autonomous rover operations. It provides a single unified command-line interface (`grizzly.py`) for all operations including installation, building, running, and testing.

### Key Features

- **Unified CLI**: Single `grizzly.py` command for all operations (install, build, run, test)
- **Cross-Platform Support**: macOS (robostack/conda), Linux (native ROS 2), and Windows
- **Lifecycle Management**: ROS 2 Lifecycle nodes for deterministic state management
- **Custom Interfaces**: Dedicated message definitions in `grizzly_interfaces`
- **Modular Architecture**: Single-package design with core system management
- **Automated Installer**: Downloads latest releases from GitHub automatically

## Quick Start

### Installation

```bash
# Clone the repository
git clone https://github.com/WashU-Robotics-Rover/grizzly-25-26.git
cd grizzly-25-26

# Option 1: Install latest release from GitHub (recommended)
./grizzly.py install

# Option 2: Build from source
./grizzly.py build
```

### Basic Usage

```bash
# Launch the system
./grizzly.py run

# Run tests
./grizzly.py test

# Get help
./grizzly.py --help
```

## Command Reference

### Install Command
```bash
./grizzly.py install              # Download latest release and build
./grizzly.py install --force      # Force re-download even if up to date
```

### Build Command
```bash
./grizzly.py build                # Standard build
./grizzly.py build --clean        # Clean build (removes build/ install/ log/)
./grizzly.py build --release      # Build in Release mode
./grizzly.py build --symlink-install  # Use symlinks for faster iteration
./grizzly.py build --packages-select grizzly_stack  # Build specific packages
```

### Run Command
```bash
./grizzly.py run                  # Launch the Grizzly minimal system
./grizzly.py run --debug          # Launch with debug output
```

### Test Command
```bash
./grizzly.py test                 # Run all tests
./grizzly.py test -v              # Verbose test output
./grizzly.py test --coverage      # Generate coverage report
./grizzly.py test --html          # Generate HTML coverage report
./grizzly.py test --list          # List all available tests
./grizzly.py test --specific test_basic  # Run specific test file
```

## Prerequisites

### macOS
- Anaconda/Miniconda with `ros_env` environment
- ROS 2 Humble (via robostack)
- Python 3.11

```bash
conda create -n ros_env python=3.11
conda activate ros_env
conda install -c robostack ros-humble-desktop colcon-common-extensions
```

### Linux
- Ubuntu 22.04 (Jammy)
- ROS 2 Humble (native install)
- Python 3.10+

```bash
# Follow: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
```

### Windows
- ROS 2 Humble (native install)
- Python 3.10+

## Architecture

### Repository Structure

```
grizzly-25-26/
├── grizzly.py                   # ⭐ Unified CLI - single entry point
├── docs/                        # Documentation
│   ├── DEVELOPMENT.md          # Development guide
│   ├── STATE_MACHINE_GUIDE.md  # State machine reference
│   ├── TESTING.md              # Testing guide
│   └── TEST_DETAILS.md         # Detailed test information
├── grizzly_interfaces/          # Custom ROS 2 message definitions
│   ├── msg/
│   │   ├── NodeStatus.msg
│   │   ├── OperationalState.msg
│   │   └── PerceptionState.msg
│   └── srv/
│       └── ChangeState.srv
└── grizzly_stack/              # Main software stack
    ├── config/
    │   └── core.yaml           # System configuration
    ├── launch/
    │   └── grizzly_minimal.launch.py
    ├── src/grizzly_stack/
    │   └── core/
    │       └── system_manager.py
    └── test/                   # Test suite
        ├── test_basic.py
        └── test_system_manager.py
```

### System Components

#### System Manager
ROS 2 Lifecycle Node that manages the rover's operational state machine.

**States**: STARTUP, STANDBY, AUTONOMOUS, MANUAL, EMERGENCY, ERROR, SHUTDOWN

**Topics**:
- `/system/state` - Current operational state (1 Hz)
- `/system/health` - Health status messages

**Services**:
- `/system/change_state` - Request state transitions

#### Custom Messages

**OperationalState**: System operational state with timestamp and description

**PerceptionState**: Integrated state estimation
- Pose with covariance (position & orientation)
- Twist with covariance (linear & angular velocity)
- Terrain classification (unknown/flat/sand/rocky)

**NodeStatus**: Individual node health reporting

For detailed state machine documentation, see [docs/STATE_MACHINE_GUIDE.md](docs/STATE_MACHINE_GUIDE.md).

## Usage Examples

### Launch the System
```bash
./grizzly.py run
```

### Monitor System State
```bash
ros2 topic echo /system/state
```

### Change Operational State
```bash
# Change to AUTONOMOUS mode
ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
  "{requested_state: 2, reason: 'Starting autonomous mission'}"

# Emergency stop
ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
  "{requested_state: 4, reason: 'EMERGENCY STOP'}"
```

### Lifecycle Management
```bash
# Get current lifecycle state
ros2 lifecycle get /system_manager

# Manually control lifecycle
ros2 lifecycle set /system_manager configure
ros2 lifecycle set /system_manager activate
```

## Configuration

Configuration files are located in `grizzly_stack/config/`:

```yaml
# core.yaml
system_manager:
  ros__parameters:
    health_rate_hz: 1.0  # Health status publish rate (Hz)
```

Modify configuration files and relaunch (no rebuild required).

## Documentation

- **[DEVELOPMENT.md](docs/DEVELOPMENT.md)** - Development workflow, coding guidelines, and contribution process
- **[STATE_MACHINE_GUIDE.md](docs/STATE_MACHINE_GUIDE.md)** - Operational state machine reference and usage
- **[TESTING.md](docs/TESTING.md)** - Testing guide and test suite documentation
- **[TEST_DETAILS.md](docs/TEST_DETAILS.md)** - Detailed test implementation information

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

# Check if node is running
ros2 node list

# Enable debug output
./grizzly.py run --debug
```

For detailed troubleshooting, see [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md).

## Current Implementation Status

✅ **Implemented:**
- Cross-platform CLI tool (`grizzly.py`)
- System manager with lifecycle management
- Operational state machine (7 states)
- Custom ROS 2 interfaces
- Automated testing (21 tests)
- Configuration management

🚧 **Planned:**
- Perception subsystem
- Planning subsystem
- Control subsystem
- Telemetry system

## Contributing

This is an internal project for the WashU Robotics Rover Team.

### Contribution Workflow

1. Create a feature branch: `git checkout -b feature/your-feature-name`
2. Make your changes and test: `./grizzly.py test`
3. Commit and push: `git push origin feature/your-feature-name`
4. Create a Pull Request on GitHub

See [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) for detailed guidelines.

## Project Information

- **Team**: WashU Robotics Rover Team
- **Repository**: https://github.com/WashU-Robotics-Rover/grizzly-25-26
- **Academic Year**: 2025-26
- **ROS 2 Distribution**: Humble
- **License**: MIT

## Contact

- **Email**: huindam@gmail.com
- **GitHub**: https://github.com/WashU-Robotics-Rover

---

**Go Bears! 🐻**
