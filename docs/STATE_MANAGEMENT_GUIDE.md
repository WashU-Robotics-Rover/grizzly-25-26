# State Management Guide

## Overview

The Grizzly Robotics Stack uses **two complementary state management systems** to provide robust, deterministic control of the rover's behavior:

1. **Lifecycle States** - ROS 2 standard for node resource management
2. **Operational States** - Custom states for mission-level coordination

This guide explains both systems, their interactions, and best practices for state management.

## State Management Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    State Management Layers                       │
│                                                                  │
│  ┌────────────────────────────────────────────────────────┐   │
│  │              Mission/Operational Layer                  │   │
│  │         (System Manager - Operational States)           │   │
│  │                                                          │   │
│  │  STARTUP → STANDBY ⇄ AUTONOMOUS ⇄ MANUAL               │   │
│  │              ↓                                           │   │
│  │         EMERGENCY / ERROR / SHUTDOWN                    │   │
│  └─────────────────────┬────────────────────────────────────┘   │
│                        │ Controls                                │
│                        ↓                                         │
│  ┌────────────────────────────────────────────────────────┐   │
│  │              Node Lifecycle Layer                       │   │
│  │          (ROS 2 Lifecycle Management)                   │   │
│  │                                                          │   │
│  │  Unconfigured → Inactive ⇄ Active → Finalized          │   │
│  └────────────────────────────────────────────────────────┘   │
│                                                                  │
│  Communication: ROS 2 Services, Topics, Parameters              │
└─────────────────────────────────────────────────────────────────┘
```

## Lifecycle States (ROS 2 Standard)

### Purpose

Lifecycle states manage **node-level resources** - when to allocate memory, start/stop timers, create publishers, etc. They ensure deterministic startup and shutdown.

### State Definitions

| State | ID | Description | Resources |
|-------|----|-----------| -------| 
| **Unconfigured** | 1 | Initial state, no resources | None |
| **Inactive** | 2 | Configured, paused | Publishers created |
| **Active** | 3 | Running normally | Publishers + timers |
| **Finalized** | 4 | Shutting down | Being destroyed |

### State Machine

```
                    configure
  Unconfigured ─────────────→ Inactive
       ↑                         │  ↑
       │                         │  │
       │       cleanup           │  │
       └─────────────────────────┘  │
       │                            │
       │                         activate
       │                            │
       │                            ↓
       └──────────────────────── Active
              deactivate

  Any State ────shutdown───→ Finalized
```

### Lifecycle Callbacks

Each lifecycle transition triggers a callback where the node can:
- Allocate/release resources
- Initialize/cleanup connections
- Validate state changes
- Return SUCCESS or FAILURE

**Example Implementation**:

```python
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn, State

class MyNode(LifecycleNode):
    def on_configure(self, state: State):
        """Allocate resources."""
        self._pub = self.create_publisher(String, 'topic', 10)
        self.get_logger().info('Configured')
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: State):
        """Start processing."""
        self._timer = self.create_timer(1.0, self.callback)
        self.get_logger().info('Activated')
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State):
        """Pause processing."""
        self.destroy_timer(self._timer)
        self._timer = None
        self.get_logger().info('Deactivated')
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: State):
        """Release resources."""
        self.destroy_publisher(self._pub)
        self._pub = None
        self.get_logger().info('Cleaned up')
        return TransitionCallbackReturn.SUCCESS
```

### Controlling Lifecycle States

**Via Command Line**:
```bash
# Configure node
ros2 lifecycle set /node_name configure

# Activate node
ros2 lifecycle set /node_name activate

# Check current state
ros2 lifecycle get /node_name

# Deactivate node
ros2 lifecycle set /node_name deactivate
```

**Via Service Call (Programmatic)**:
```python
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

# Create client
client = node.create_client(ChangeState, '/node_name/change_state')

# Prepare request
request = ChangeState.Request()
request.transition.id = Transition.TRANSITION_ACTIVATE

# Call service
future = client.call_async(request)
```

## Operational States (Grizzly Custom)

### Purpose

Operational states manage **mission-level behavior** - what the rover should be doing overall. They coordinate multiple subsystems and determine which nodes should be active.

### State Definitions

| State | ID | Description | Active Subsystems |
|-------|----|-----------| -----------------|
| **STARTUP** | 0 | System initializing | Core only |
| **STANDBY** | 1 | Ready, awaiting commands | Core only |
| **AUTONOMOUS** | 2 | Self-driving navigation | Core + Perception + Navigation |
| **MANUAL** | 3 | Operator teleop control | Core + Telemetry |
| **EMERGENCY** | 4 | Emergency stop engaged | Core + Perception |
| **ERROR** | 5 | System error state | Core only |
| **SHUTDOWN** | 6 | Shutting down | None (deactivating) |

### State Machine

```
              ┌─────────────────┐
              │     STARTUP     │
              └────────┬────────┘
                       │
                       ↓
              ┌─────────────────┐
         ┌───→│     STANDBY     │←───┐
         │    └────────┬────────┘    │
         │             │              │
         │      ┌──────┴──────┐      │
         │      ↓             ↓      │
    ┌────┴─────────┐   ┌─────────────┴──┐
    │  AUTONOMOUS  │⇄  │     MANUAL      │
    └────┬─────────┘   └────────┬────────┘
         │                      │
         └──────┐      ┐────────┘
                ↓      ↓
         ┌──────────────────┐
         │    EMERGENCY     │
         └────────┬─────────┘
                  │
                  ↓
         ┌──────────────────┐
         │      ERROR       │
         └────────┬─────────┘
                  │
                  ↓
         ┌──────────────────┐
         │    SHUTDOWN      │
         └──────────────────┘
```

### Valid Transitions

The system enforces valid transitions to prevent dangerous state changes:

```python
_valid_transitions = {
    STARTUP:     [STANDBY, ERROR, EMERGENCY],
    STANDBY:     [AUTONOMOUS, MANUAL, SHUTDOWN, EMERGENCY],
    AUTONOMOUS:  [STANDBY, MANUAL, EMERGENCY, ERROR],
    MANUAL:      [STANDBY, AUTONOMOUS, EMERGENCY, ERROR],
    EMERGENCY:   [STANDBY],  # Must assess before other states
    ERROR:       [STANDBY, SHUTDOWN],
    SHUTDOWN:    []  # Terminal state
}
```

### Changing Operational States

**Via Service Call**:
```bash
# Request autonomous mode
ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
  "{requested_state: 2, reason: 'Starting autonomous mission'}"

# Request standby mode
ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
  "{requested_state: 1, reason: 'Mission complete'}"
```

**Response**:
```yaml
success: True
message: "State changed from STANDBY to AUTONOMOUS"
```

**Monitoring Current State**:
```bash
# Subscribe to state topic
ros2 topic echo /system/state

# Will show:
state: 2  # AUTONOMOUS
```

## State Interaction: Lifecycle ↔ Operational

### How They Work Together

The System Manager uses operational states to control lifecycle states of managed nodes through the **Layer Manager**. Instead of managing individual nodes, the system uses **layer-based organization**:

```python
def _on_state_transition(self, old_state, new_state):
    """Delegate layer management based on operational state."""
    
    # Delegate to Layer Manager, which handles nodes organized by layers
    self._layer_manager.handle_state_transition(old_state, new_state)
```

The Layer Manager automatically:
- Activates required layers based on operational state
- Deactivates layers no longer needed
- Manages all nodes within each layer together
- Handles emergency/shutdown states immediately

### State Mapping Example (Layer-Based)

| Operational State | System Manager | Active Layers | Node States |
|------------------|----------------|---------------|-------------|
| STARTUP | Active | None | All nodes inactive |
| STANDBY | Active | None | All nodes inactive |
| AUTONOMOUS | Active | **perception, planning, control** | All nodes in these layers active |
| MANUAL | Active | **perception, planning, control** | All nodes in these layers active |
| EMERGENCY | Active | None | All operational layers deactivated immediately |
| ERROR | Active | None | All operational layers deactivated |
| SHUTDOWN | Active | None | All layers deactivated |

### Typical State Flow

```
System Boot
     │
     ↓
┌────────────────┐
│ Lifecycle      │  Lifecycle Manager handles initial startup
│ Manager        │  - Configures system_manager
│ Startup        │  - Activates system_manager
└───────┬────────┘  - Configures perception_node (leaves inactive)
        │
        ↓
┌────────────────┐
│ Operational    │  System Manager controls operational behavior
│ State:         │  - Starts in STARTUP
│ STARTUP        │  - Validates system health
└───────┬────────┘  - Transitions to STANDBY when ready
        │
        ↓
┌────────────────┐
│ Operational    │  Awaiting mission commands
│ State:         │  All nodes inactive except core
│ STANDBY        │
└───────┬────────┘
        │
        │  (User requests autonomous mode)
        ↓
┌────────────────┐
│ Operational    │  Layer Manager activates layers:
│ State:         │  - perception layer (all nodes)
│ AUTONOMOUS     │  - planning layer (all nodes)
│                │  - control layer (all nodes)
└───────┬────────┘
        │
        │  (Mission complete)
        ↓
┌────────────────┐
│ Operational    │  Layer Manager deactivates layers:
│ State:         │  - perception layer: active → inactive
│ STANDBY        │  - planning layer: active → inactive
│                │  - control layer: active → inactive
└────────────────┘
```

## Layer Management

### Overview

The **Layer Manager** provides a scalable architecture for managing nodes organized into logical layers. Instead of managing individual nodes, the system groups related nodes together and activates/deactivates entire layers based on operational state.

**Benefits**:
- **Scalability**: Adding new nodes to existing layers doesn't require modifying system manager code
- **Organization**: Related nodes are grouped logically (perception, planning, control, etc.)
- **Simplicity**: System manager delegates lifecycle control to layer manager
- **Maintainability**: Layer configuration is centralized in YAML

### Layer Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    System Manager                           │
│                                                              │
│  Operational State Changes                                  │
│         │                                                    │
│         ↓                                                    │
│  ┌──────────────────────────────────────┐                   │
│  │         Layer Manager                │                   │
│  │                                       │                   │
│  │  ┌──────────────────────────────┐    │                   │
│  │  │  State-Layer Mapping         │    │                   │
│  │  │  AUTONOMOUS → [perception,   │    │                   │
│  │  │               planning,      │    │                   │
│  │  │               control]      │    │                   │
│  │  └──────────────────────────────┘    │                   │
│  │         │                            │                   │
│  │         ↓                            │                   │
│  │  ┌──────────────────────────────┐    │                   │
│  │  │  Layer Configuration         │    │                   │
│  │  │  perception: [perception_node]│   │                   │
│  │  │  planning: [planner_node]    │    │                   │
│  │  │  control: [control_node]    │    │                   │
│  │  └──────────────────────────────┘    │                   │
│  │         │                            │                   │
│  │         ↓                            │                   │
│  │  Lifecycle Transitions (per node)   │                   │
│  └──────────────────────────────────────┘                   │
└─────────────────────────────────────────────────────────────┘
```

### Layer Configuration

Layers are configured in `grizzly_stack/config/layers.yaml`:

```yaml
layers:
  perception:
    nodes:
      - perception_node
      - sensor_fusion_node  # Multiple nodes per layer
    startup_order: 1
    description: "Perception and sensor processing layer"
  
  planning:
    nodes:
      - planner_node
      - path_finder_node
    startup_order: 2
    description: "Path and behavior planning layer"
  
  control:
    nodes:
      - control_node
    startup_order: 3
    description: "Motor and actuator control layer"
```

**Configuration Fields**:
- `nodes`: List of node names in this layer (can be multiple)
- `startup_order`: Order in which layers are configured during startup (lower = earlier)
- `description`: Human-readable description of the layer's purpose

### State-Layer Mapping

The Layer Manager maps operational states to required layers:

| Operational State | Active Layers | Purpose |
|------------------|----------------|---------|
| **STARTUP** | None | System initializing, no operational layers |
| **STANDBY** | None | Ready but idle, save resources |
| **AUTONOMOUS** | perception, planning, control | Full autonomous operation |
| **MANUAL** | perception, planning, control | Manual control with full sensing |
| **EMERGENCY** | None | Immediate deactivation of all operational layers |
| **ERROR** | None | Error state, deactivate operational layers |
| **SHUTDOWN** | None | System shutting down |

**Custom Mapping**: The state-layer mapping is defined in `LayerManager._state_layer_mapping`. To change which layers are active in each state, modify this mapping.

### Adding Nodes to Layers

To add a new node to an existing layer, simply add it to the `nodes` list in `layers.yaml`:

```yaml
layers:
  perception:
    nodes:
      - perception_node
      - new_sensor_node  # ← Add here
    startup_order: 1
```

**No code changes needed!** The layer manager automatically:
- Detects the new node during startup (via lifecycle manager)
- Manages its lifecycle transitions along with other nodes in the layer
- Activates/deactivates it based on operational state

### Creating New Layers

To create a new layer:

1. **Add to `layers.yaml`**:
```yaml
layers:
  navigation:
    nodes:
      - navigation_node
      - localization_node
    startup_order: 2  # Between perception (1) and control (3)
    description: "Navigation and localization layer"
```

2. **Update state-layer mapping** in `LayerManager._state_layer_mapping`:
```python
self._state_layer_mapping = {
    OperationalState.AUTONOMOUS: ['perception', 'planning', 'navigation', 'control'],
    # ... other states
}
```

3. **Rebuild and test**:
```bash
./grizzly.py build
./grizzly.py test
./grizzly.py run
```

### Layer Lifecycle Flow

```
Operational State: STANDBY → AUTONOMOUS
        │
        ↓
Layer Manager: Determine required layers
        │
        ├─→ perception layer (not active) → activate
        ├─→ planning layer (not active) → activate
        └─→ control layer (not active) → activate
        │
        ↓
For each layer:
        │
        ├─→ For each node in layer:
        │       ├─→ Check current state (cached)
        │       ├─→ If inactive → activate
        │       └─→ If unconfigured → configure then activate
        │
        ↓
All nodes in required layers active
```

### Emergency Handling

The Layer Manager has special handling for emergency and shutdown states:

```python
if new_state == OperationalState.EMERGENCY:
    # Deactivate ALL layers immediately, bypassing normal logic
    self._deactivate_all_layers()
    return  # Skip normal layer management
```

This ensures immediate response to emergency conditions without waiting for normal layer transitions.

### Best Practices for Layers

1. **Logical Grouping**: Group nodes that work together or have similar purposes
2. **Startup Order**: Order layers by dependencies (e.g., perception before planning)
3. **State Mapping**: Consider which layers are needed for each operational state
4. **Layer Size**: Keep layers focused - don't put too many unrelated nodes in one layer
5. **Documentation**: Add clear descriptions to help other developers understand layer purposes

### Example: Adding a Teleoperation Layer

```yaml
# layers.yaml
layers:
  teleoperation:
    nodes:
      - teleop_node
      - joystick_node
    startup_order: 4
    description: "Teleoperation and manual control layer"
```

```python
# In LayerManager._state_layer_mapping
OperationalState.MANUAL: ['perception', 'teleoperation'],  # Manual mode needs teleop
```

## Best Practices

### 1. State Tracking

Always track state internally to avoid invalid transitions:

```python
class MyNode(LifecycleNode):
    def __init__(self):
        super().__init__('my_node')
        self._node_states = {}  # Track managed node states
    
    def _activate_node(self, node_name):
        current_state = self._node_states.get(node_name, 'unknown')
        
        # Only activate if inactive
        if current_state != 'inactive':
            self.get_logger().info(
                f'Skipping activation - {node_name} is {current_state}'
            )
            return
        
        # Proceed with activation...
        self._node_states[node_name] = 'active'
```

### 2. Error Handling in Callbacks

Always return appropriate status and handle errors:

```python
def on_configure(self, state: State):
    try:
        # Configuration logic
        self._pub = self.create_publisher(String, 'topic', 10)
        
        # Validate configuration
        if not self._validate_config():
            self.get_logger().error('Invalid configuration')
            return TransitionCallbackReturn.FAILURE
        
        return TransitionCallbackReturn.SUCCESS
        
    except Exception as e:
        self.get_logger().error(f'Configuration failed: {e}')
        return TransitionCallbackReturn.FAILURE
```

### 3. Resource Management

Follow the lifecycle pattern strictly:

```python
# ❌ BAD: Creating resources in __init__
def __init__(self):
    super().__init__('node')
    self._timer = self.create_timer(1.0, callback)  # Don't do this!
    self._pub = self.create_publisher(...)  # Or this!

# ✅ GOOD: Create resources in lifecycle callbacks
def __init__(self):
    super().__init__('node')
    self._timer = None
    self._pub = None

def on_configure(self, state):
    self._pub = self.create_publisher(...)  # Resources here
    return TransitionCallbackReturn.SUCCESS

def on_activate(self, state):
    self._timer = self.create_timer(...)  # Timers here
    return TransitionCallbackReturn.SUCCESS
```

### 4. State Validation

Validate requested state transitions:

```python
def _request_state_change(self, requested_state, reason):
    """Validate and execute state change."""
    
    # Check if transition is valid
    valid_next_states = self._valid_transitions.get(self._current_state, [])
    
    if requested_state not in valid_next_states:
        return ChangeStateResponse(
            success=False,
            message=f'Invalid transition from {self._current_state} to {requested_state}'
        )
    
    # Proceed with state change...
```

### 5. Graceful Degradation

Handle subsystem failures gracefully:

```python
def _activate_lifecycle_node(self, node_name):
    """Activate a node with error handling."""
    
    try:
        # Check state first
        if not self._is_node_ready(node_name):
            self.get_logger().warn(f'{node_name} not ready')
            # Continue anyway - don't block other subsystems
            return
        
        # Request activation
        response = self._send_activation_request(node_name)
        
        if not response.success:
            self.get_logger().error(f'Failed to activate {node_name}')
            # Log error but don't crash - rover can operate in degraded mode
            
    except Exception as e:
        self.get_logger().error(f'Exception activating {node_name}: {e}')
        # Continue operating with remaining subsystems
```

### 6. State Monitoring

Log state transitions for debugging:

```python
def _transition_to_state(self, new_state, reason):
    """Transition to new operational state with logging."""
    
    old_state = self._current_state
    
    self.get_logger().info(
        f'State transition: {old_state} -> {new_state} '
        f'(reason: {reason})'
    )
    
    # Record in history
    self._state_history.append({
        'from': old_state,
        'to': new_state,
        'reason': reason,
        'timestamp': self.get_clock().now()
    })
    
    # Execute transition
    self._handle_state_transition(old_state, new_state)
    self._current_state = new_state
    
    # Publish state change
    self._publish_state()
```

## Testing State Management

### Unit Testing Lifecycle Callbacks

```python
def test_on_configure_success(self):
    """Test successful configuration."""
    node = MyLifecycleNode()
    mock_state = Mock()
    
    result = node.on_configure(mock_state)
    
    assert result == TransitionCallbackReturn.SUCCESS
    assert node._pub is not None
```

### Integration Testing State Transitions

```python
def test_full_state_sequence(self):
    """Test complete operational state sequence."""
    manager = SystemManager()
    
    # Start in STARTUP
    assert manager.get_current_state() == OperationalState.STARTUP
    
    # Transition to STANDBY
    response = manager.request_state_change(
        OperationalState.STANDBY,
        'Testing'
    )
    assert response.success == True
    assert manager.get_current_state() == OperationalState.STANDBY
```

### Manual Testing

```bash
# 1. Start system
ros2 launch grizzly_stack grizzly_minimal.launch.py

# 2. Monitor states in separate terminals
ros2 lifecycle get /system_manager
ros2 lifecycle get /perception_node
ros2 topic echo /system/state

# 3. Request state changes
ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
  "{requested_state: 2, reason: 'Testing'}"

# 4. Verify subsystems activated
ros2 lifecycle get /perception_node  # Should be "active"
```

## Troubleshooting

### Node Stuck in Wrong State

**Problem**: Node won't transition to expected state

**Diagnosis**:
```bash
# Check current lifecycle state
ros2 lifecycle get /node_name

# Check available transitions
ros2 lifecycle list /node_name

# Check node logs
ros2 run grizzly_stack node_name  # Look for errors
```

**Solutions**:
- Ensure lifecycle callback returns SUCCESS
- Check for exceptions in callback code
- Verify resource availability (sensors, connections)

### Invalid State Transition Rejected

**Problem**: State change request rejected

**Diagnosis**:
```bash
# Check current operational state
ros2 topic echo /system/state --once

# Try state change
ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
  "{requested_state: X, reason: 'Testing'}"

# Read response message
```

**Solutions**:
- Review valid transitions in state machine
- Transition through intermediate states if needed
- Check for EMERGENCY or ERROR state blocking transitions

### Subsystems Not Activating

**Problem**: Operational state changes but subsystems don't activate

**Diagnosis**:
```bash
# Check System Manager logs
ros2 node info /system_manager

# Verify node is managed
ros2 service call /system_manager/list_nodes

# Check lifecycle state
ros2 lifecycle get /perception_node
```

**Solutions**:
- Ensure node is in `_managed_nodes` list
- Verify lifecycle services are available
- Check for state tracking issues

## Related Documentation

- [Lifecycle Manager](LIFECYCLE_MANAGER.md) - Event-based node orchestration
- [Lifecycle Implementation Summary](LIFECYCLE_IMPLEMENTATION_SUMMARY.md) - Technical details
- [Lifecycle Diagram](LIFECYCLE_DIAGRAM.md) - Visual state machine
- [Perception System](PERCEPTION_SYSTEM.md) - Example lifecycle node
- [State Machine Guide](STATE_MACHINE_GUIDE.md) - Implementation patterns

## References

- [ROS 2 Managed Nodes](https://design.ros2.org/articles/node_lifecycle.html)
- [ROS 2 Lifecycle](https://github.com/ros2/demos/tree/master/lifecycle)
- [Grizzly State Machine Implementation](../grizzly_stack/src/grizzly_stack/core/system_manager.py)
