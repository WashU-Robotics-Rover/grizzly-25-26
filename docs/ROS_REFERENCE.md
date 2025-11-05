# ROS 2 Reference

Complete ROS 2 reference documentation for the Grizzly Rover software stack, including nodes, topics, services, messages, and configuration.

## Table of Contents

- [ROS 2 Nodes](#ros-2-nodes)
- [Topics and Services](#topics-and-services)
- [Messages and Interfaces](#messages-and-interfaces)
- [Layer Configuration](#layer-configuration)
- [System Configuration](#system-configuration)

## ROS 2 Nodes

### Core Nodes

#### `system_manager`

**Type**: Lifecycle Node  
**Description**: Central orchestration node managing the rover's operational state machine.

**Published Topics:**
- `/system/state` (grizzly_interfaces/msg/OperationalState) - Current operational state at 1 Hz
- `/system/health` (std_msgs/msg/String) - Health status messages at 1 Hz

**Subscribed Topics:** None

**Services:**
- `/system/change_state` (grizzly_interfaces/srv/ChangeState) - Request state transitions

**Parameters:**
- `health_rate_hz` (double, default: 1.0) - Health status publish rate

**Lifecycle States:** Unconfigured → Inactive → Active

**Operational States:**
- `STARTUP` (0) - System initializing
- `STANDBY` (1) - Ready but not active
- `AUTONOMOUS` (2) - Autonomous operation mode
- `MANUAL` (3) - Manual/teleoperation mode
- `EMERGENCY` (4) - Emergency stop engaged
- `ERROR` (5) - System error state
- `SHUTDOWN` (6) - System shutting down

**Usage:**
```bash
ros2 run grizzly_stack system_manager
```

---

#### `lifecycle_manager`

**Type**: Regular Node  
**Description**: Orchestration node that manages startup and lifecycle transitions of all lifecycle-managed nodes.

**Published Topics:** None

**Subscribed Topics:** None

**Services:**
- Queries lifecycle services of managed nodes

**Parameters:**
- `managed_nodes` (list of strings) - List of node names to manage
- `startup_timeout` (double, default: 30.0) - Maximum time to wait for node startup

**Usage:**
```bash
ros2 run grizzly_stack lifecycle_manager
```

---

#### `layer_manager`

**Type**: Utility Class (used by System Manager)  
**Description**: Manages lifecycle transitions for nodes organized into logical layers.

**Configuration:** Defined in `grizzly_stack/config/layers.yaml`

**Features:**
- Layer-based node organization
- Automatic activation/deactivation based on operational state
- Async lifecycle transitions
- State-layer mapping

---

### Perception Nodes

#### `perception_node`

**Type**: Lifecycle Node  
**Description**: Template perception node for sensor processing and state estimation.

**Published Topics:**
- `/perception/state` (grizzly_interfaces/msg/PerceptionState) - Perception state at configurable rate

**Subscribed Topics:**
- Sensor topics (camera, lidar, IMU, etc.)

**Parameters:**
- `publish_rate_hz` (double, default: 10.0) - Perception state publish rate
- `sensor_enabled` (bool, default: true) - Enable/disable sensor processing

**Lifecycle States:** Unconfigured → Inactive → Active

**Usage:**
```bash
ros2 run grizzly_stack perception_node
```

---

#### `central_perception`

**Type**: Lifecycle Node  
**Description**: Central sensor processing and state estimation node.

**Published Topics:**
- `/perception/state` (grizzly_interfaces/msg/PerceptionState) - Integrated perception state

**Subscribed Topics:**
- Sensor data topics (camera, lidar, IMU)

**Parameters:**
- `publish_rate_hz` (double) - State publish rate
- `frame_id` (string, default: "base_link") - Reference frame

**Usage:**
```bash
ros2 run grizzly_stack central_perception
```

---

### Planning Nodes

#### `planner_node`

**Type**: Lifecycle Node  
**Description**: Template planner node for path and behavior planning.

**Published Topics:**
- Planning output topics (paths, commands)

**Subscribed Topics:**
- `/perception/state` (grizzly_interfaces/msg/PerceptionState) - Perception state
- `/system/state` (grizzly_interfaces/msg/OperationalState) - System state

**Parameters:**
- Planning-specific parameters

**Usage:**
```bash
ros2 run grizzly_stack planner_node
```

---

#### `drive_planner`

**Type**: Lifecycle Node  
**Description**: High-level path and behavior planning for the drive subsystem.

**Usage:**
```bash
ros2 run grizzly_stack drive_planner
```

---

#### `arm_planner`

**Type**: Lifecycle Node  
**Description**: Planning for arm manipulation operations.

**Usage:**
```bash
ros2 run grizzly_stack arm_planner
```

---

#### `science_planner`

**Type**: Lifecycle Node  
**Description**: Planning for science operations.

**Usage:**
```bash
ros2 run grizzly_stack science_planner
```

---

### Control Nodes

#### `control_node`

**Type**: Lifecycle Node  
**Description**: Template control node for motor and actuator control.

**Published Topics:**
- Control feedback topics

**Subscribed Topics:**
- Planning command topics
- `/system/state` (grizzly_interfaces/msg/OperationalState) - System state

**Parameters:**
- Control-specific parameters

**Usage:**
```bash
ros2 run grizzly_stack control_node
```

---

## Topics and Services

### System Topics

#### `/system/state`

**Type**: `grizzly_interfaces/msg/OperationalState`  
**Publisher**: `system_manager`  
**Rate**: 1 Hz (configurable)  
**Description**: Current operational state of the rover system.

**Message Fields:**
- `state` (uint8) - Current state value
- `timestamp` (builtin_interfaces/Time) - Time of state
- `description` (string) - Human-readable description

**Usage:**
```bash
ros2 topic echo /system/state
```

---

#### `/system/health`

**Type**: `std_msgs/msg/String`  
**Publisher**: `system_manager`  
**Rate**: 1 Hz (configurable)  
**Description**: Health status messages from the system.

**Usage:**
```bash
ros2 topic echo /system/health
```

---

### Perception Topics

#### `/perception/state`

**Type**: `grizzly_interfaces/msg/PerceptionState`  
**Publisher**: `perception_node`, `central_perception`  
**Rate**: Configurable (default: 10 Hz)  
**Description**: Integrated perception state including pose, velocity, and terrain classification.

**Message Fields:**
- `header` (std_msgs/Header) - Timestamp and frame information
- `pose` (geometry_msgs/PoseWithCovariance) - Position and orientation with uncertainty
- `twist` (geometry_msgs/TwistWithCovariance) - Linear and angular velocity with uncertainty
- `terrain_class` (uint8) - Terrain classification (0=unknown, 1=flat, 2=sand, 3=rocky)

**Usage:**
```bash
ros2 topic echo /perception/state
```

---

### Services

#### `/system/change_state`

**Type**: `grizzly_interfaces/srv/ChangeState`  
**Server**: `system_manager`  
**Description**: Request a change to the operational state.

**Request:**
- `requested_state` (uint8) - Desired state (use OperationalState constants)
- `reason` (string) - Optional reason for state change

**Response:**
- `success` (bool) - True if state change was accepted
- `current_state` (uint8) - Current state after the request
- `message` (string) - Human-readable response message

**Usage:**
```bash
# Change to AUTONOMOUS mode
ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
  "{requested_state: 2, reason: 'Starting autonomous mission'}"

# Trigger EMERGENCY stop
ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
  "{requested_state: 4, reason: 'EMERGENCY STOP'}"
```

---

## Messages and Interfaces

### Custom Messages

All custom messages are defined in the `grizzly_interfaces` package.

#### `OperationalState.msg`

Represents the current operational mode of the rover system.

```
# State constants
uint8 STARTUP = 0      # System initializing
uint8 STANDBY = 1      # Ready but not active
uint8 AUTONOMOUS = 2   # Autonomous operation mode
uint8 MANUAL = 3       # Manual/teleoperation mode
uint8 EMERGENCY = 4    # Emergency stop engaged
uint8 ERROR = 5        # System error state
uint8 SHUTDOWN = 6     # System shutting down

# Current state
uint8 state

# Timestamp of last state change
builtin_interfaces/Time timestamp

# Human-readable state description
string description
```

**Python Usage:**
```python
from grizzly_interfaces.msg import OperationalState

# Create state message
state_msg = OperationalState()
state_msg.state = OperationalState.AUTONOMOUS
state_msg.description = "Autonomous operation active"
state_msg.timestamp = self.get_clock().now().to_msg()
```

---

#### `PerceptionState.msg`

Represents the complete perception state of the rover at a given moment.

```
# Header contains timestamp and frame information
std_msgs/Header header

# Robot's position and orientation with uncertainty
geometry_msgs/PoseWithCovariance pose

# Robot's linear and angular velocity with uncertainty
geometry_msgs/TwistWithCovariance twist

# Terrain classification
# 0 = unknown, 1 = flat, 2 = sand, 3 = rocky
uint8 terrain_class
```

**Python Usage:**
```python
from grizzly_interfaces.msg import PerceptionState
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance

perception_msg = PerceptionState()
perception_msg.header.frame_id = "base_link"
perception_msg.pose = PoseWithCovariance()
perception_msg.twist = TwistWithCovariance()
perception_msg.terrain_class = PerceptionState.TERRAIN_FLAT
```

---

#### `NodeStatus.msg`

Individual node health reporting.

```
# Node name
string node_name

# Health status
bool healthy

# Status message
string status_message

# Timestamp
builtin_interfaces/Time timestamp
```

---

### Services

#### `ChangeState.srv`

Service to change the operational state of the system.

**Request:**
```
uint8 requested_state  # Desired state (use OperationalState constants)
string reason          # Optional reason for state change
```

**Response:**
```
bool success           # True if state change was accepted
uint8 current_state    # Current state after the request
string message         # Human-readable response message
```

**Python Usage:**
```python
from grizzly_interfaces.srv import ChangeState
from grizzly_interfaces.msg import OperationalState

# Create service client
client = self.create_client(ChangeState, '/system/change_state')

# Wait for service
client.wait_for_service()

# Create request
request = ChangeState.Request()
request.requested_state = OperationalState.AUTONOMOUS
request.reason = "Starting autonomous mission"

# Send request
future = client.call_async(request)
```

---

## Layer Configuration

Layers are defined in `grizzly_stack/config/layers.yaml`. This file organizes nodes into logical groups for lifecycle management.

### Configuration Format

```yaml
layers:
  layer_name:
    nodes:
      - node_name_1
      - node_name_2
    startup_order: 1  # Optional: order for startup sequence
    description: "Layer description"
```

### Default Layers

#### `perception`
- **Nodes**: `perception_node`
- **Startup Order**: 1
- **Description**: Perception and sensor processing layer
- **Active When**: AUTONOMOUS, MANUAL (optional)

#### `planning`
- **Nodes**: `planner_node`
- **Startup Order**: 2
- **Description**: Path and behavior planning layer
- **Active When**: AUTONOMOUS, MANUAL

#### `control`
- **Nodes**: `control_node`
- **Startup Order**: 3
- **Description**: Motor and actuator control layer
- **Active When**: AUTONOMOUS, MANUAL

### Adding New Layers

To add a new layer:

1. Add the layer definition to `layers.yaml`:
```yaml
layers:
  navigation:
    nodes:
      - navigation_node
      - localization_node
    startup_order: 2
    description: "Navigation and localization layer"
```

2. The Layer Manager will automatically manage nodes in this layer during operational state transitions.

### State-Layer Mapping

Layers are automatically activated/deactivated based on operational state:

- **AUTONOMOUS/MANUAL**: Activates perception, planning, control layers
- **STANDBY/STARTUP**: No operational layers active
- **EMERGENCY/ERROR/SHUTDOWN**: Deactivates all operational layers immediately

---

## System Configuration

System configuration is defined in `grizzly_stack/config/core.yaml`.

### Configuration Format

```yaml
node_name:
  ros__parameters:
    parameter_name: value
```

### System Manager Configuration

```yaml
system_manager:
  ros__parameters:
    health_rate_hz: 1.0  # Health status publish rate (Hz)
```

### Launch Configuration

Launch files are located in `grizzly_stack/launch/`:

- **`grizzly_minimal.launch.py`** - Main launch file for the Grizzly system

**Usage:**
```bash
ros2 launch grizzly_stack grizzly_minimal.launch.py
```

Or via CLI:
```bash
./grizzly.py run
```

---

## Command-Line Tools

### ROS 2 CLI

#### List Nodes
```bash
ros2 node list
```

#### Get Node Info
```bash
ros2 node info /system_manager
```

#### List Topics
```bash
ros2 topic list
ros2 topic echo /topic_name
ros2 topic hz /topic_name
```

#### List Services
```bash
ros2 service list
ros2 service type /service_name
ros2 service call /service_name service_type "{request: value}"
```

#### Lifecycle Management
```bash
ros2 lifecycle get /node_name
ros2 lifecycle set /node_name configure
ros2 lifecycle set /node_name activate
ros2 lifecycle set /node_name deactivate
ros2 lifecycle set /node_name cleanup
ros2 lifecycle set /node_name shutdown
```

### Grizzly CLI

See the main [README.md](README.md#command-reference) for `grizzly.py` command reference.

---

## Best Practices

### Subscribing to Topics

```python
from grizzly_interfaces.msg import OperationalState

self.subscription = self.create_subscription(
    OperationalState,
    '/system/state',
    self.state_callback,
    10  # Queue size
)
```

### Publishing Topics

```python
from grizzly_interfaces.msg import OperationalState

self.publisher = self.create_publisher(
    OperationalState,
    '/system/state',
    10  # Queue size
)

# Publish
msg = OperationalState()
msg.state = OperationalState.AUTONOMOUS
msg.timestamp = self.get_clock().now().to_msg()
msg.description = "Autonomous mode"
self.publisher.publish(msg)
```

### Calling Services

```python
from grizzly_interfaces.srv import ChangeState

client = self.create_client(ChangeState, '/system/change_state')
client.wait_for_service()

request = ChangeState.Request()
request.requested_state = OperationalState.AUTONOMOUS
request.reason = "Starting mission"

future = client.call_async(request)
rclpy.spin_until_future_complete(self, future)
response = future.result()
```

---

## See Also

- [State Machine Guide](STATE_MACHINE_GUIDE.md) - Detailed state machine reference
- [State Management Guide](STATE_MANAGEMENT_GUIDE.md) - Comprehensive state management
- [Development Guide](DEVELOPMENT.md) - Development workflow and practices

