# Grizzly Rover 2025-26

A ROS 2 robotics software stack for the WashU Robotics Rover Team's Grizzly rover platform. This repository provides a unified software architecture with automated release management and cross-platform build support for macOS and Linux.

## Overview

The Grizzly stack is designed as a modular, lifecycle-managed system for autonomous rover operations. It features an automated installer that downloads the latest releases from GitHub and builds them with platform-specific configurations.

### Key Features

- **Automated Installer**: Python-based installer (`installer.py`) that downloads and builds the latest GitHub releases
- **Cross-Platform Support**: macOS (robostack/conda) and Linux (native ROS 2) build configurations
- **Lifecycle Management**: ROS 2 Lifecycle nodes for deterministic state management
- **Custom Interfaces**: Dedicated message definitions in `grizzly_interfaces`
- **Modular Architecture**: Single-package design with core system management
- **Release Packaging**: Automated tarball generation with version tracking

## Quick Start

### Option 1: Automated Installation (Recommended)

The installer automatically downloads the latest pre-release from GitHub and builds it:

```bash
# Run the installer
python3 installer.py
```

The installer will:
1. Check for the latest release on GitHub
2. Download release assets to `downloads/`
3. Extract and build the workspace automatically
4. Configure platform-specific build settings

### Option 2: Manual Build

For development or manual control:

```bash
# Clone the repository
git clone https://github.com/WashU-Robotics-Rover/grizzly-25-26.git
cd grizzly-25-26

# Build based on your platform
./build.sh              # macOS (requires conda ros_env)
./install_grizzly.sh    # Linux (native ROS 2 Humble)
```

## Architecture

### Repository Structure

```
grizzly-25-26/
├── installer.py                 # Automated installer (downloads & builds releases)
├── build.sh                     # macOS build script (conda/robostack)
├── run.sh                       # Launch script for macOS
├── install_grizzly.sh          # Linux build script (native ROS 2)
├── README_INSTALL.md           # Release installation instructions
├── downloads/                   # Downloaded release assets
│   ├── grizzly-source-*.tar.gz # Release tarball
│   ├── version.json            # Version tracking
│   ├── SHA256SUMS              # Release checksums
│   └── src/                    # Extracted source code
├── grizzly_interfaces/          # Custom ROS 2 message definitions
│   ├── package.xml             # Package manifest
│   ├── CMakeLists.txt          # Build configuration
│   └── msg/
│       └── PerceptionState.msg # Pose, twist, and terrain data
└── grizzly_stack/              # Main software stack
    ├── package.xml             # Package manifest
    ├── setup.py                # Python package setup
    ├── launch/
    │   └── grizzly_minimal.launch.py  # Minimal system launcher
    ├── config/
    │   └── core.yaml           # System configuration
    └── src/grizzly_stack/
        └── core/
            └── system_manager.py  # Lifecycle-managed health monitor
```

### System Components

#### Installer System
- **installer.py**: Automated release management
  - Fetches latest pre-release from GitHub API
  - Version comparison and caching
  - Cross-platform build orchestration
  - Platform detection (macOS/Linux)

#### Core Package (`grizzly_stack`)
- **System Manager**: ROS 2 Lifecycle Node
  - Health monitoring and status reporting
  - Lifecycle states: Unconfigured → Inactive → Active → Finalized
  - Configurable health check rate (1.0 Hz default)
  - Publishes to `/system/health` topic

#### Custom Interfaces (`grizzly_interfaces`)
- **PerceptionState Message**: Integrated state estimation
  - Pose with covariance (position & orientation)
  - Twist with covariance (linear & angular velocity)
  - Terrain classification (unknown/flat/sand/rocky)

### Launch System

The minimal launch file (`grizzly_minimal.launch.py`) demonstrates ROS 2 Lifecycle management:
- Launches `system_manager` as a Lifecycle Node
- Automatic state transitions via `TimerAction`
- Configure transition at 1 second
- Activate transition at 2 seconds

## Prerequisites

### macOS (robostack/conda)
- **Anaconda/Miniconda** with `ros_env` environment
- **ROS 2 Humble** installed via robostack
- **Python 3.11** in conda environment
- **colcon** build tool

Setup conda environment:
```bash
# Create environment (if not exists)
conda create -n ros_env python=3.11
conda activate ros_env

# Install ROS 2 via robostack
conda install -c robostack ros-humble-desktop
conda install -c robostack colcon-common-extensions
```

### Linux (Native ROS 2)
- **Ubuntu 22.04** (Jammy)
- **ROS 2 Humble** (from apt packages)
- **Python 3.10+**
- **colcon**, `rosdep`

Install ROS 2 Humble:
```bash
# Follow official guide:
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
```

## Installation Details

### macOS Installation

The macOS build uses conda environment with explicit Python paths:

```bash
# Activate conda environment
eval "$(conda shell.zsh hook)"
conda activate ros_env

# Source ROS 2 environment
source /opt/anaconda3/envs/ros_env/setup.zsh

# Build with explicit Python configuration
./build.sh
```

The `build.sh` script automatically:
- Activates the `ros_env` conda environment
- Sources the ROS 2 setup from conda
- Configures CMake with explicit Python paths to avoid version conflicts
- Sources the workspace after building

### Linux Installation

The Linux build uses native ROS 2 Humble:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the workspace
./install_grizzly.sh

# Optional flags:
# --no-rosdep : Skip dependency installation
# --release   : Use Release build type
```

The `install_grizzly.sh` script:
- Checks for ROS 2 Humble and colcon
- Runs `rosdep` to install dependencies (unless `--no-rosdep`)
- Builds with `RelWithDebInfo` or `Release` mode
- Provides setup instructions

## Usage

### Launch the System

**macOS (using convenience script):**
```bash
./run.sh
```

**Manual launch (both platforms):**
```bash
# Source the workspace first
source install/setup.bash  # Linux
source install/setup.zsh   # macOS

# Launch the minimal system
ros2 launch grizzly_stack grizzly_minimal.launch.py
```

### Run Individual Nodes

The system manager is currently the only implemented node:

```bash
# Run system manager directly
ros2 run grizzly_stack system_manager

# Check system health
ros2 topic echo /system/health
```

### Lifecycle Node Management

The system manager is a Lifecycle Node. You can manually control its state:

```bash
# Get current state
ros2 lifecycle get /system_manager

# Transition states manually
ros2 lifecycle set /system_manager configure
ros2 lifecycle set /system_manager activate
ros2 lifecycle set /system_manager deactivate
ros2 lifecycle set /system_manager cleanup
```

## Configuration

Configuration files are located in `grizzly_stack/config/`:

**`core.yaml`** - System Manager parameters:
```yaml
system_manager:
  ros__parameters:
    health_rate_hz: 1.0  # Health status publish rate (Hz)
```

Modify configuration files and relaunch (no rebuild required).

## Custom Messages

### PerceptionState

Defined in `grizzly_interfaces/msg/PerceptionState.msg`:

```msg
std_msgs/Header header                      # Timestamp and frame
geometry_msgs/PoseWithCovariance pose       # Position & orientation
geometry_msgs/TwistWithCovariance twist     # Linear & angular velocity
uint8 terrain_class                         # Terrain classification
```

**Terrain class values:**
- `0` = unknown
- `1` = flat (optimal driving)
- `2` = sand (reduced traction)
- `3` = rocky (careful navigation)

## Development

### Development Workflow

```bash
# Make changes to Python files
# ...

# Rebuild (with symlink for faster iteration)
./build.sh --symlink-install

# Source and test
source install/setup.zsh  # or setup.bash
ros2 launch grizzly_stack grizzly_minimal.launch.py
```

### Project Structure Conventions

- **Lifecycle Nodes**: All major system components use Lifecycle Nodes for deterministic startup/shutdown
- **Configuration**: Parameters defined in YAML files under `config/`
- **Launch Files**: Use `TimerAction` for automatic lifecycle transitions
- **Messages**: Custom messages in `grizzly_interfaces` package

### Testing

```bash
# Run tests
colcon test

# View test results
colcon test-result --verbose

# Run specific package tests
colcon test --packages-select grizzly_stack
```

## Release Management

### Creating a Release

The repository includes automated release packaging:

1. **Tag and push**: Create a version tag and push to GitHub
2. **GitHub Actions**: Builds the release tarball automatically
3. **Assets**: Release includes:
   - `grizzly-source-*.tar.gz` - Source tarball
   - `version.json` - Version metadata
   - `SHA256SUMS` - Checksums for verification
   - `README_INSTALL.md` - Installation instructions

### Using Releases

The installer automatically handles releases:

```bash
# Download and build latest pre-release
python3 installer.py
```

The installer:
- Checks local version vs. GitHub latest
- Downloads only if new version available
- Extracts to `downloads/src/`
- Builds using platform-specific scripts

## Troubleshooting

### macOS Issues

**Python environment errors:**
```bash
# Verify conda environment
conda info --envs
conda activate ros_env
which python  # Should show /opt/anaconda3/envs/ros_env/bin/python

# Check Python version
python --version  # Should be 3.11.x

# If paths don't match, update build.sh paths
```

**Build failures:**
```bash
# Clean and rebuild
rm -rf build/ install/ log/
./build.sh
```

### Linux Issues

**ROS 2 not found:**
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Verify installation
ros2 --version
```

**Missing dependencies:**
```bash
# Update rosdep and install dependencies
rosdep update
rosdep install --from-paths . --ignore-src -r -y
```

**Build errors:**
```bash
# Clean build
rm -rf build/ install/ log/
./install_grizzly.sh
```

### Installer Issues

**Download fails:**
- Check internet connection
- Verify GitHub repository access: `https://github.com/WashU-Robotics-Rover/grizzly-25-26`
- Check for API rate limiting

**Version mismatch:**
```bash
# Force re-download
rm -rf downloads/
python3 installer.py
```

**Build fails after download:**
```bash
# Manually build from downloads
cd downloads/src/
./build.sh              # macOS
./install_grizzly.sh    # Linux
```

## Project Information

- **Team**: WashU Robotics Rover Team
- **Repository**: https://github.com/WashU-Robotics-Rover/grizzly-25-26
- **License**: MIT
- **Version**: 0.1.0
- **Academic Year**: 2025-26
- **ROS 2 Distribution**: Humble

### Current Implementation Status

✅ **Implemented:**
- Automated installer with GitHub release integration
- Cross-platform build system (macOS/Linux)
- Core system manager with lifecycle management
- Custom message interface (PerceptionState)
- Minimal launch configuration
- Configuration system via YAML

🚧 **Planned:**
- Perception subsystem (sensor processing, state estimation)
- Planning subsystem (drive, arm, science planners)
- Control subsystem (low-level controllers)
- Telemetry system (data logging and monitoring)

## Contributing

This is an internal project for the WashU Robotics Rover Team.

**For team members:**

1. **Clone and setup**:
   ```bash
   git clone https://github.com/WashU-Robotics-Rover/grizzly-25-26.git
   cd grizzly-25-26
   ```

2. **Create a feature branch**:
   ```bash
   git checkout -b feature/your-feature-name
   ```

3. **Develop and test**:
   ```bash
   ./build.sh --symlink-install
   # Test your changes
   ```

4. **Commit and push**:
   ```bash
   git add .
   git commit -m "Description of changes"
   git push origin feature/your-feature-name
   ```

5. **Create Pull Request**: Submit PR on GitHub for review

### Code Style

- **Python**: Follow PEP 8 style guidelines
- **Documentation**: Include docstrings for all classes and functions
- **Comments**: Explain *why*, not just *what*
- **ROS 2 Conventions**: Follow ROS 2 naming and package structure standards

## Contact

For questions or issues, contact the WashU Robotics Rover Team:
- **Email**: huindam@gmail.com
- **GitHub**: https://github.com/WashU-Robotics-Rover

---

**Go Bears! 🐻**
