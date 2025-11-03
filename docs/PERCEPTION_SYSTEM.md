# Perception System

## Overview

The **Perception System** provides sensor data processing, environmental awareness, and state estimation for the Grizzly Rover. It's implemented as a lifecycle-managed node that integrates with the System Manager for operational state coordination.

## Architecture

### System Components

```
┌───────────────────────────────────────────────────────────────┐
│                      Perception System                         │
│                                                                │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │                  Perception Node                        │ │
│  │                 (Lifecycle Managed)                     │ │
│  │                                                          │ │
│  │  ┌─────────────┐  ┌──────────────┐  ┌──────────────┐  │ │
│  │  │   Sensor    │  │   Processing │  │    State     │  │ │
│  │  │  Interface  │→ │    Pipeline  │→ │  Publishing  │  │ │
│  │  └─────────────┘  └──────────────┘  └──────────────┘  │ │
│  │                                                          │ │
│  │  Configuration:          Active when:                   │ │
│  │  - Sensor type          - AUTONOMOUS mode               │ │
│  │  - Processing rate      - MANUAL mode (optional)        │ │
│  │  - Enable/disable       - System Manager requests       │ │
│  └─────────────────────────────────────────────────────────┘ │
│                                                                │
│  Managed by: System Manager                                   │
│  Coordinated by: Lifecycle Manager (startup only)             │
└───────────────────────────────────────────────────────────────┘
```

### Data Flow

```
                    ┌──────────────┐
                    │   Sensors    │
                    │ (Camera/Lidar)
                    └───────┬──────┘
                            │
                            ↓
                ┌─────────────────────┐
                │  Perception Node    │
                │   (when Active)     │
                └─────────┬───────────┘
                          │
        ┌─────────────────┼─────────────────┐
        ↓                 ↓                 ↓
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│   Obstacle   │  │  Environment │  │   Position   │
│  Detection   │  │    Mapping   │  │  Estimation  │
└──────────────┘  └──────────────┘  └──────────────┘
        │                 │                 │
        └─────────────────┼─────────────────┘
                          ↓
                  ┌──────────────┐
                  │   Planning   │
                  │  Navigation  │
                  │   Control    │
                  └──────────────┘
```

## Lifecycle States

The perception node follows the ROS 2 lifecycle state machine:

### State Definitions

| State | Description | Resources | Processing |
|-------|-------------|-----------|------------|
| **Unconfigured** | Initial state after creation | None | None |
| **Inactive** | Configured but not running | Publishers created | Paused |
| **Active** | Fully operational | Publishers + timers | Running |
| **Finalized** | Shutting down | Destroyed | Stopped |

### State Transitions

```
Unconfigured ──configure──→ Inactive ──activate──→ Active
     ↑                          ↑                      │
     │                          │                      │
     └────────cleanup───────────┘                      │
     │                                                  │
     └───────────────────deactivate────────────────────┘
```

### Lifecycle Callbacks

#### `on_configure(state)`

**Purpose**: Allocate resources and set up publishers.

**Actions**:
- Load configuration parameters (sensor type, rate, enabled flag)
- Create publisher for perception data (`/perception/data`)
- Log configuration details
- Validate parameters

**Returns**: `TransitionCallbackReturn.SUCCESS` or `FAILURE`

**Example**:
```python
def on_configure(self, state: State):
    self.get_logger().info('Configuring Perception Node...')
    
    # Get parameters
    self.sensor_type = self.get_parameter('sensor_type').value
    self.processing_rate = self.get_parameter('processing_rate_hz').value
    
    # Create publisher
    self._pub = self.create_publisher(String, '/perception/data', 10)
    
    self.get_logger().info(
        f'Perception configured: sensor={self.sensor_type}, '
        f'rate={self.processing_rate}Hz'
    )
    
    return TransitionCallbackReturn.SUCCESS
```

#### `on_activate(state)`

**Purpose**: Start processing and publishing data.

**Actions**:
- Create timer for periodic processing
- Begin sensor data acquisition
- Start publishing perception data

**Returns**: `TransitionCallbackReturn.SUCCESS` or `FAILURE`

**Example**:
```python
def on_activate(self, state: State):
    self.get_logger().info('Activating Perception Node...')
    
    # Calculate timer period from rate
    period = 1.0 / self.processing_rate
    
    # Create processing timer
    self._timer = self.create_timer(period, self._timer_callback)
    
    self.get_logger().info('Perception node activated - processing started')
    return TransitionCallbackReturn.SUCCESS
```

#### `on_deactivate(state)`

**Purpose**: Stop processing while keeping resources allocated.

**Actions**:
- Cancel and destroy timer
- Stop sensor acquisition
- Keep publishers alive for quick restart

**Returns**: `TransitionCallbackReturn.SUCCESS` or `FAILURE`

#### `on_cleanup(state)`

**Purpose**: Release all allocated resources.

**Actions**:
- Destroy publishers
- Release sensor interfaces
- Clear internal buffers

**Returns**: `TransitionCallbackReturn.SUCCESS` or `FAILURE`

## Configuration

### Parameters

The perception node is configured via `/config/perception.yaml`:

```yaml
perception_node:
  ros__parameters:
    # Sensor type: camera, lidar, radar, stereo
    sensor_type: "camera"
    
    # Processing frequency in Hz
    processing_rate_hz: 2.0
    
    # Enable/disable perception system
    enabled: true
    
    # Advanced parameters
    max_detection_range: 10.0  # meters
    confidence_threshold: 0.75
    debug_visualization: false
```

### Parameter Details

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sensor_type` | string | "camera" | Type of primary sensor |
| `processing_rate_hz` | double | 2.0 | Processing frequency |
| `enabled` | bool | true | Enable perception in launch |
| `max_detection_range` | double | 10.0 | Maximum detection distance (m) |
| `confidence_threshold` | double | 0.75 | Minimum confidence for detections |
| `debug_visualization` | bool | false | Enable debug visualizations |

### Updating Configuration

To update perception configuration:

1. Edit `config/perception.yaml`
2. Rebuild the workspace: `colcon build --packages-select grizzly_stack`
3. Restart the system: `ros2 launch grizzly_stack grizzly_minimal.launch.py`

## Operational Integration

### Interaction with System Manager

The perception node is managed by the System Manager based on operational states:

| Operational State | Perception State | Rationale |
|-------------------|------------------|-----------|
| **STARTUP** | Inactive | System initializing |
| **STANDBY** | Inactive | Conserve resources |
| **AUTONOMOUS** | Active | Required for navigation |
| **MANUAL** | Inactive* | Operator controls directly |
| **EMERGENCY** | Active | Maintain awareness |
| **ERROR** | Inactive | Safe mode |
| **SHUTDOWN** | Inactive | System shutting down |

_* Manual mode can optionally activate perception for operator assistance_

### State Change Flow

```
System Manager                    Perception Node
      │                                 │
      │  State: AUTONOMOUS              │
      ├────────────────────────────────→│
      │  Activate Request               │
      │                                 │
      │                           on_activate()
      │                           Start Processing
      │                                 │
      │  ←────────────────────────────┤
      │  Activation Confirmed           │
      │                                 │
      │  State: STANDBY                 │
      ├────────────────────────────────→│
      │  Deactivate Request             │
      │                                 │
      │                         on_deactivate()
      │                          Stop Processing
      │                                 │
      │  ←────────────────────────────┤
      │  Deactivation Confirmed         │
```

## Usage

### Starting Perception

The perception node starts automatically if `enabled: true` in configuration:

```bash
# Source workspace
source install/setup.bash

# Launch system (includes perception if enabled)
ros2 launch grizzly_stack grizzly_minimal.launch.py
```

### Manual Lifecycle Control

For testing or manual control:

```bash
# Configure perception node
ros2 lifecycle set /perception_node configure

# Activate perception node
ros2 lifecycle set /perception_node activate

# Check current state
ros2 lifecycle get /perception_node

# Deactivate perception node
ros2 lifecycle set /perception_node deactivate
```

### Monitoring Perception Data

```bash
# View perception output
ros2 topic echo /perception/data

# Monitor processing rate
ros2 topic hz /perception/data

# View node info
ros2 node info /perception_node
```

## Sensor Integration

### Supported Sensor Types

The perception system supports multiple sensor types:

1. **Camera** - Visual perception, object detection
2. **Lidar** - 3D point cloud, obstacle detection
3. **Radar** - Long-range detection, velocity estimation
4. **Stereo** - Depth perception, 3D reconstruction

### Adding New Sensors

To integrate a new sensor:

1. **Create sensor interface** in `perception/sensors/`
2. **Add processing pipeline** for sensor data type
3. **Update configuration** with new sensor type
4. **Register in perception node** sensor factory

Example structure:
```python
# perception/sensors/my_sensor.py
class MySensor:
    def initialize(self):
        """Setup sensor hardware/drivers."""
        pass
    
    def read(self):
        """Read and return sensor data."""
        pass
    
    def process(self, data):
        """Process raw sensor data."""
        pass
```

## Processing Pipeline

### Data Flow

1. **Acquisition** - Read raw sensor data
2. **Preprocessing** - Filter, normalize, calibrate
3. **Feature Extraction** - Identify relevant features
4. **Detection/Estimation** - Object detection, pose estimation
5. **Publishing** - Publish results on ROS topics

### Timer Callback

The main processing loop executes at the configured rate:

```python
def _timer_callback(self):
    """Main processing callback - runs at processing_rate_hz."""
    
    # 1. Read sensor data
    raw_data = self.sensor.read()
    
    # 2. Process data
    processed = self.process_sensor_data(raw_data)
    
    # 3. Publish results
    msg = String()
    msg.data = f"Perception data: {processed}"
    self._pub.publish(msg)
    
    self.get_logger().info('Published perception data', throttle_duration_sec=5.0)
```

## Testing

### Unit Tests

See `test/test_perception_node.py` for comprehensive tests:

- Lifecycle state transitions
- Configuration parameter handling
- Processing callback execution
- Error handling

Run tests:
```bash
# Run all perception tests
colcon test --packages-select grizzly_stack --pytest-args -k test_perception

# Run specific test
colcon test --packages-select grizzly_stack --pytest-args -k test_on_activate_success
```

### Integration Testing

Test perception with the full system:

```bash
# 1. Launch system
ros2 launch grizzly_stack grizzly_minimal.launch.py

# 2. In another terminal, trigger autonomous mode
ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
  "{requested_state: 2, reason: 'Testing perception'}"

# 3. Verify perception is active
ros2 lifecycle get /perception_node  # Should show "active [3]"

# 4. Monitor perception output
ros2 topic echo /perception/data
```

## Troubleshooting

### Perception Node Won't Activate

**Symptoms**: Node stuck in inactive state

**Possible Causes**:
- Configuration error in `perception.yaml`
- Sensor hardware not available
- System Manager not requesting activation

**Solutions**:
```bash
# Check configuration
cat install/grizzly_stack/share/grizzly_stack/config/perception.yaml

# Manually activate to see error
ros2 lifecycle set /perception_node activate

# Check system manager state
ros2 topic echo /system/state
```

### No Perception Data Published

**Symptoms**: Topic exists but no messages

**Possible Causes**:
- Node not in active state
- Processing rate too low
- Sensor data unavailable

**Solutions**:
```bash
# Verify node is active
ros2 lifecycle get /perception_node

# Check topic connections
ros2 topic info /perception/data

# Increase processing rate in config
# Set processing_rate_hz: 10.0
```

### High CPU Usage

**Symptoms**: Perception consuming excessive resources

**Solutions**:
- Reduce `processing_rate_hz` in configuration
- Disable `debug_visualization`
- Optimize processing algorithms
- Consider moving to inactive when not needed

## Future Enhancements

Potential improvements for the perception system:

1. **Multi-Sensor Fusion** - Combine camera + lidar data
2. **ML Integration** - Add neural network-based detection
3. **Dynamic Rate Adjustment** - Auto-adjust based on system load
4. **Sensor Health Monitoring** - Detect sensor failures
5. **Advanced Visualization** - RViz2 integration for debugging
6. **Odometry Integration** - Combine with wheel encoders
7. **Map Building** - SLAM capabilities

## Related Documentation

- [Lifecycle Manager](LIFECYCLE_MANAGER.md)
- [Perception Template](PERCEPTION_TEMPLATE.md)
- [State Management Guide](STATE_MANAGEMENT_GUIDE.md)
- [System Manager](../README.md#system-manager)
- [Testing Guide](TESTING.md)
