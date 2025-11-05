# Grizzly Rover Documentation

Welcome to the Grizzly Rover 2025-26 documentation. This documentation provides comprehensive guides for developing, testing, and understanding the Grizzly robotics software stack.

## Quick Navigation

### Getting Started
- **[Development Guide](DEVELOPMENT.md)** - Set up your development environment, learn the workflow, and contribution guidelines
- **[Testing Guide](TESTING.md)** - How to write and run tests for the Grizzly stack

### System Architecture
- **[State Management Guide](STATE_MANAGEMENT_GUIDE.md)** - Comprehensive guide to lifecycle and operational state management
- **[State Machine Guide](STATE_MACHINE_GUIDE.md)** - Detailed reference for the operational state machine
- **[Lifecycle Manager](LIFECYCLE_MANAGER.md)** - Understanding the lifecycle orchestration system
- **[Perception System](PERCEPTION_SYSTEM.md)** - Documentation for the perception subsystem

### Testing
- **[Test Details](TEST_DETAILS.md)** - Detailed information about test implementations

## Overview

The Grizzly stack is a ROS 2 robotics software stack designed as a modular, lifecycle-managed system for autonomous rover operations. It provides a single unified command-line interface (`grizzly.py`) for all operations including installation, building, running, and testing.

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

## System Components

### System Manager
ROS 2 Lifecycle Node that manages the rover's operational state machine and delegates lifecycle control to the Layer Manager.

**States**: STARTUP, STANDBY, AUTONOMOUS, MANUAL, EMERGENCY, ERROR, SHUTDOWN

### Layer Manager
Manages lifecycle transitions for nodes organized into logical layers. Nodes are grouped into layers (e.g., perception, planning, control) and activated/deactivated together based on operational state.

### Lifecycle Manager
Orchestration node that manages the startup and lifecycle transitions of all lifecycle-managed nodes, providing event-based, deterministic node initialization.

## Project Information

- **Team**: WashU Robotics Rover Team
- **Repository**: https://github.com/WashU-Robotics-Rover/grizzly-25-26
- **Academic Year**: 2025-26
- **ROS 2 Distribution**: Humble
- **License**: MIT

---

**Go Bears! 🐻**

