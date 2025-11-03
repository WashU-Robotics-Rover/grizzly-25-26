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

The System Manager uses operational states to control lifecycle states of managed nodes:

```python
def _handle_state_transition(self, old_state, new_state):
    """Manage subsystem lifecycle based on operational state."""
    
    if new_state == OperationalState.AUTONOMOUS:
        # Activate perception for autonomous navigation
        self._activate_lifecycle_nodes(['perception_node'])
    
    elif new_state == OperationalState.STANDBY:
        # Deactivate operational nodes to save resources
        self._deactivate_lifecycle_nodes(['perception_node'])
```

### State Mapping Example

| Operational State | System Manager | Perception Node | Navigation Node |
|------------------|----------------|-----------------|-----------------|
| STARTUP | Active | Inactive | Inactive |
| STANDBY | Active | Inactive | Inactive |
| AUTONOMOUS | Active | **Active** | **Active** |
| MANUAL | Active | Inactive | Inactive |
| EMERGENCY | Active | **Active** | Inactive |

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
│ Operational    │  System Manager activates subsystems:
│ State:         │  - perception_node: inactive → active
│ AUTONOMOUS     │  - navigation: inactive → active
└───────┬────────┘
        │
        │  (Mission complete)
        ↓
┌────────────────┐
│ Operational    │  System Manager deactivates subsystems:
│ State:         │  - perception_node: active → inactive
│ STANDBY        │  - navigation: active → inactive
└────────────────┘
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
