# Development Guide

A comprehensive guide for developing and contributing to the Grizzly Rover software stack.

## Development Environment Setup

### Prerequisites

#### macOS (robostack/conda)
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

#### Linux (Native ROS 2)
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

## Development Workflow

### Fast Iteration with Symlink Install

For rapid development, use symlink install to avoid copying Python files:

```bash
# Make changes to Python files
# ...

# Rebuild (with symlink for faster iteration)
./grizzly.py build --symlink-install

# Source and test
source install/setup.zsh  # macOS
source install/setup.bash # Linux
ros2 launch grizzly_stack grizzly_minimal.launch.py
```

### Clean Build

When you need a completely clean build:

```bash
./grizzly.py build --clean
```

This removes `build/`, `install/`, and `log/` directories before building.

### Testing Changes

Always test your changes before committing:

```bash
# Run all tests
./grizzly.py test

# Run tests with coverage
./grizzly.py test --coverage

# Run specific tests
./grizzly.py test --specific test_system_manager
```

## Project Structure Conventions

### Package Layout

```
grizzly_stack/
├── package.xml              # ROS 2 package manifest
├── setup.py                 # Python package setup
├── setup.cfg                # Python package configuration
├── config/                  # Configuration files (YAML)
│   └── core.yaml           # System parameters
├── launch/                  # Launch files
│   └── grizzly_minimal.launch.py
├── src/grizzly_stack/       # Python source code
│   ├── __init__.py
│   └── core/
│       ├── __init__.py
│       └── system_manager.py
└── test/                    # Unit tests
    ├── test_basic.py
    └── test_system_manager.py
```

### Code Organization

- **Lifecycle Nodes**: All major system components use Lifecycle Nodes for deterministic startup/shutdown
- **Configuration**: Parameters defined in YAML files under `config/`
- **Launch Files**: Use `TimerAction` for automatic lifecycle transitions
- **Messages**: Custom messages in `grizzly_interfaces` package

## Architecture Guidelines

### Lifecycle Node Pattern

All major nodes should implement the ROS 2 Lifecycle pattern:

```python
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn

class MyNode(LifecycleNode):
    def __init__(self):
        super().__init__('my_node')
        # Initialize but don't start resources
    
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure resources (open files, set up connections)."""
        self.get_logger().info('Configuring...')
        # Setup code here
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Start active work (create timers, start threads)."""
        self.get_logger().info('Activating...')
        # Start timers, publishers, etc.
        return super().on_activate(state)
    
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Stop active work but keep resources."""
        self.get_logger().info('Deactivating...')
        # Stop timers, threads, etc.
        return super().on_deactivate(state)
    
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Clean up resources."""
        self.get_logger().info('Cleaning up...')
        # Release resources
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Final shutdown."""
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS
```

### State Machine Pattern

Use the SystemManager's operational state machine for coordinating rover modes:

```python
from grizzly_interfaces.msg import OperationalState
from grizzly_interfaces.srv import ChangeState

class MyControllerNode(Node):
    def __init__(self):
        super().__init__('my_controller')
        
        # Subscribe to state updates
        self.state_sub = self.create_subscription(
            OperationalState,
            '/system/state',
            self.state_callback,
            10
        )
        
        # Client to request state changes
        self.state_client = self.create_client(
            ChangeState,
            '/system/change_state'
        )
    
    def state_callback(self, msg):
        """React to state changes."""
        if msg.state == OperationalState.AUTONOMOUS:
            self.enable_autonomous_mode()
        elif msg.state == OperationalState.EMERGENCY:
            self.emergency_stop()
```

## Building and Testing

### Build Commands

```bash
# Standard build
./grizzly.py build

# Clean build
./grizzly.py build --clean

# Release mode (optimizations)
./grizzly.py build --release

# Symlink install (faster iteration)
./grizzly.py build --symlink-install

# Build specific packages
./grizzly.py build --packages-select grizzly_stack
```

### Test Commands

```bash
# Run all tests
./grizzly.py test

# Verbose output
./grizzly.py test -v

# Coverage report
./grizzly.py test --coverage

# HTML coverage report
./grizzly.py test --html

# Run specific test
./grizzly.py test --specific test_system_manager

# List all tests
./grizzly.py test --list
```

### Manual Testing

```bash
# Launch the system
./grizzly.py run

# In another terminal, interact with nodes
ros2 node list
ros2 topic list
ros2 service list

# Monitor system state
ros2 topic echo /system/state

# Change state
ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
  "{requested_state: 2, reason: 'Testing'}"

# Check lifecycle state
ros2 lifecycle get /system_manager
ros2 lifecycle set /system_manager configure
ros2 lifecycle set /system_manager activate
```

## Platform-Specific Notes

### macOS Build Details

The macOS build uses conda environment with explicit Python paths:
- Requires `ros_env` conda environment with robostack
- Automatically activates conda and sources ROS 2
- Configures CMake with explicit Python paths to avoid version conflicts
- Uses zsh as the shell

### Linux Build Details

The Linux build uses native ROS 2 Humble:
- Requires ROS 2 Humble installed at `/opt/ros/humble`
- Runs `rosdep` to install dependencies automatically
- Builds with `RelWithDebInfo` by default
- Uses bash as the shell

### Windows Build Details

The Windows build uses native ROS 2 Humble:
- Requires ROS 2 Humble installed in standard Windows location
- Uses `colcon` directly
- No automatic dependency installation

## Contributing Workflow

### 1. Create a Feature Branch

```bash
git checkout -b feature/your-feature-name
```

### 2. Make Your Changes

Follow the code style guidelines below.

### 3. Test Your Changes

```bash
./grizzly.py test
./grizzly.py build
./grizzly.py run
```

### 4. Commit and Push

```bash
git add .
git commit -m "Description of changes"
git push origin feature/your-feature-name
```

### 5. Create Pull Request

Submit a PR on GitHub for review.

## Code Style Guidelines

### Python Style (PEP 8)

- **Indentation**: 4 spaces (no tabs)
- **Line length**: 100 characters max
- **Naming**:
  - Functions/variables: `snake_case`
  - Classes: `PascalCase`
  - Constants: `UPPER_SNAKE_CASE`
  - Private methods: `_leading_underscore`

### Documentation

- **Docstrings**: All classes and functions should have docstrings
- **Format**: Use Google-style or NumPy-style docstrings

Example:
```python
def calculate_velocity(position_data, time_delta):
    """
    Calculate velocity from position data.
    
    Args:
        position_data (np.ndarray): Position measurements
        time_delta (float): Time between measurements
    
    Returns:
        np.ndarray: Calculated velocity vector
    
    Raises:
        ValueError: If time_delta is zero or negative
    """
    if time_delta <= 0:
        raise ValueError("time_delta must be positive")
    return position_data / time_delta
```

### Comments

- Explain *why*, not just *what*
- Use `#` for inline comments
- Keep comments up to date with code changes

### ROS 2 Conventions

- **Node Names**: `snake_case` (e.g., `system_manager`)
- **Topic Names**: `/namespace/topic_name` (e.g., `/system/state`)
- **Service Names**: `/namespace/service_name` (e.g., `/system/change_state`)
- **Parameter Names**: `snake_case` (e.g., `health_rate_hz`)

## Configuration Management

### Parameter Files

Parameters are stored in YAML files under `grizzly_stack/config/`:

```yaml
system_manager:
  ros__parameters:
    health_rate_hz: 1.0
```

### Loading Parameters in Nodes

```python
class MyNode(LifecycleNode):
    def __init__(self):
        super().__init__('my_node')
        
        # Declare and get parameters
        self.declare_parameter('my_param', 1.0)
        my_value = self.get_parameter('my_param').value
```

### Launch File Parameters

```python
def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('grizzly_stack'),
        'config',
        'core.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='grizzly_stack',
            executable='system_manager',
            name='system_manager',
            parameters=[config_file]
        )
    ])
```

## Debugging Tips

### Enable Debug Logging

```bash
# Launch with debug output
./grizzly.py run --debug

# Or manually:
ros2 launch grizzly_stack grizzly_minimal.launch.py --log-level debug
```

### ROS 2 Debugging Tools

```bash
# Check node graph
ros2 node list
ros2 node info /system_manager

# Monitor topics
ros2 topic list
ros2 topic echo /system/state
ros2 topic hz /system/state

# Check services
ros2 service list
ros2 service type /system/change_state

# Inspect parameters
ros2 param list /system_manager
ros2 param get /system_manager health_rate_hz
```

### Python Debugging

Add breakpoints in your code:

```python
import pdb; pdb.set_trace()  # Python debugger
```

Or use VS Code's debugger with launch configurations.

## Troubleshooting

### Build Issues

**Problem**: CMake can't find Python
```bash
# macOS: Check conda environment
conda activate ros_env
which python

# Linux: Install Python dev packages
sudo apt install python3-dev
```

**Problem**: Missing dependencies
```bash
# Linux: Install dependencies
rosdep update
rosdep install --from-paths . --ignore-src -r -y

# macOS: Install via conda
conda install -c robostack <package-name>
```

### Runtime Issues

**Problem**: Node not found
```bash
# Make sure workspace is sourced
source install/setup.bash  # Linux
source install/setup.zsh   # macOS

# Check if node is installed
ros2 pkg executables grizzly_stack
```

**Problem**: Import errors
```bash
# Rebuild with symlink install
./grizzly.py build --clean --symlink-install
```

## Performance Optimization

### Build Performance

- Use `--symlink-install` for Python-only changes
- Use `--packages-select` to build only what changed
- Use `--parallel-workers N` to control parallelism

### Runtime Performance

- Profile with `ros2 run` and timing tools
- Monitor CPU/memory usage
- Use `Release` build mode for production

## Release Management

### Creating a Release

1. Update version numbers in `package.xml` files
2. Update CHANGELOG.md
3. Create and push a version tag
4. GitHub Actions will build the release automatically

### Testing a Release

```bash
# Install latest release
./grizzly.py install

# Test the installation
./grizzly.py build
./grizzly.py test
./grizzly.py run
```

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Lifecycle Nodes](https://design.ros2.org/articles/node_lifecycle.html)
- [Colcon Documentation](https://colcon.readthedocs.io/)
- [State Machine Guide](STATE_MACHINE_GUIDE.md)
- [Testing Guide](TESTING.md)

## Contact

For questions or issues:
- **Email**: huindam@gmail.com
- **GitHub**: https://github.com/WashU-Robotics-Rover

---

**Go Bears! 🐻**
