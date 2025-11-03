# Lifecycle Manager

## Overview

The **Lifecycle Manager** is an orchestration node that manages the startup and lifecycle transitions of all lifecycle-managed nodes in the Grizzly Robotics Stack. It provides event-based, deterministic node initialization that adapts to system performance rather than relying on fixed timers.

## Architecture

### Design Philosophy

Traditional ROS 2 launch files use either:
- **Fixed timers** - Inflexible, may be too short on slow systems or waste time on fast systems
- **Event handlers** - Unreliable on some platforms (especially macOS) and can create race conditions

The Lifecycle Manager solves these issues by:
1. **Detecting node availability** before attempting transitions
2. **Querying actual state** before each transition
3. **Waiting for state confirmation** after transitions
4. **Adapting to system performance** automatically
5. **Exiting cleanly** after startup is complete

### Component Responsibilities

```
┌─────────────────────────────────────────────────────────────┐
│                     Lifecycle Manager                        │
│                                                              │
│  ┌────────────────────────────────────────────────────┐    │
│  │  1. Node Detection Phase                           │    │
│  │     - Wait for lifecycle service availability      │    │
│  │     - Create service clients                       │    │
│  │     - Verify node responsiveness                   │    │
│  └────────────────────────────────────────────────────┘    │
│                           ↓                                  │
│  ┌────────────────────────────────────────────────────┐    │
│  │  2. State Query Phase                              │    │
│  │     - Query current state of each node             │    │
│  │     - Validate transition is possible              │    │
│  └────────────────────────────────────────────────────┘    │
│                           ↓                                  │
│  ┌────────────────────────────────────────────────────┐    │
│  │  3. Transition Execution Phase                     │    │
│  │     - Send transition request                      │    │
│  │     - Wait for acknowledgment                      │    │
│  └────────────────────────────────────────────────────┘    │
│                           ↓                                  │
│  ┌────────────────────────────────────────────────────┐    │
│  │  4. Verification Phase                             │    │
│  │     - Poll node state until target reached         │    │
│  │     - Timeout if state doesn't change              │    │
│  │     - Log success/failure                          │    │
│  └────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

## Startup Sequence

The Lifecycle Manager executes a carefully ordered sequence of transitions:

### Default Sequence

1. **System Manager**: `unconfigured` → `inactive` (configure)
2. **System Manager**: `inactive` → `active` (activate)
3. **Perception Node** (if detected): `unconfigured` → `inactive` (configure)

> **Note**: The perception node is configured but NOT activated by the lifecycle manager. The System Manager will activate it based on operational state transitions.

### Dynamic Detection

The manager dynamically detects which nodes are available:

```python
def build_startup_sequence(self):
    """Dynamically build startup sequence based on available nodes."""
    sequence = [
        ('system_manager', 'inactive', 'Configure System Manager'),
        ('system_manager', 'active', 'Activate System Manager'),
    ]
    
    # Check if perception node is available
    if perception_service_available():
        sequence.append(
            ('perception_node', 'inactive', 'Configure Perception Node')
        )
    
    return sequence
```

## Key Methods

### `wait_for_node(node_name, timeout_sec)`

Waits for a lifecycle node's services to become available.

**Parameters:**
- `node_name` (str): Name of the lifecycle node
- `timeout_sec` (float): Maximum time to wait (default: 30.0)

**Returns:**
- `bool`: True if node is available, False if timeout

**Example:**
```python
if manager.wait_for_node('system_manager', timeout_sec=10.0):
    print("System manager is ready!")
```

### `get_node_state(node_name)`

Queries the current lifecycle state of a node.

**Parameters:**
- `node_name` (str): Name of the lifecycle node

**Returns:**
- `str`: Current state label ('unconfigured', 'inactive', 'active', etc.) or None if failed

**Example:**
```python
state = manager.get_node_state('perception_node')
if state == 'inactive':
    print("Perception node is configured but not active")
```

### `transition_node(node_name, transition_id, target_state_label, timeout_sec)`

Transitions a node to a new state and waits for completion.

**Parameters:**
- `node_name` (str): Name of the lifecycle node
- `transition_id` (int): Transition ID from `lifecycle_msgs.msg.Transition`
- `target_state_label` (str): Expected state after transition
- `timeout_sec` (float): Maximum time to wait (default: 10.0)

**Returns:**
- `bool`: True if transition succeeded, False otherwise

**Example:**
```python
from lifecycle_msgs.msg import Transition

success = manager.transition_node(
    'perception_node',
    Transition.TRANSITION_CONFIGURE,
    'inactive',
    timeout_sec=5.0
)
```

### `execute_startup_sequence()`

Executes the complete startup sequence for all managed nodes.

**Returns:**
- `bool`: True if all transitions succeeded, False otherwise

**Process:**
1. Build dynamic startup sequence
2. Wait for all nodes to be available
3. Execute each transition in order
4. Verify each transition completed
5. Log results and exit

## Configuration

The Lifecycle Manager doesn't require configuration files. It automatically:
- Detects available nodes by checking service availability
- Adapts timeouts based on system responsiveness
- Builds the appropriate startup sequence

## Usage

### In Launch Files

```python
from launch_ros.actions import Node

lifecycle_manager_node = Node(
    package='grizzly_stack',
    executable='lifecycle_manager',
    name='lifecycle_manager',
    output='screen',
)
```

### Standalone Execution

```bash
# Source the workspace
source install/setup.bash

# Run lifecycle manager
ros2 run grizzly_stack lifecycle_manager
```

### Expected Output

```
[INFO] [lifecycle_manager]: Lifecycle Manager starting...
[INFO] [lifecycle_manager]: ============================================================
[INFO] [lifecycle_manager]: Starting Lifecycle Management Sequence
[INFO] [lifecycle_manager]: ============================================================
[INFO] [lifecycle_manager]: Perception node detected - adding to sequence
[INFO] [lifecycle_manager]: Waiting for system_manager to become available...
[INFO] [lifecycle_manager]: ✅ system_manager is available
[INFO] [lifecycle_manager]: Waiting for perception_node to become available...
[INFO] [lifecycle_manager]: ✅ perception_node is available
[INFO] [lifecycle_manager]: 
[INFO] [lifecycle_manager]: All nodes detected, starting transitions...
[INFO] [lifecycle_manager]: 
[INFO] [lifecycle_manager]: Step: Configure System Manager
[INFO] [lifecycle_manager]: Transitioning system_manager: unconfigured -> inactive
[INFO] [lifecycle_manager]: ✅ system_manager successfully transitioned to inactive
[INFO] [lifecycle_manager]: 
[INFO] [lifecycle_manager]: Step: Activate System Manager
[INFO] [lifecycle_manager]: Transitioning system_manager: inactive -> active
[INFO] [lifecycle_manager]: ✅ system_manager successfully transitioned to active
[INFO] [lifecycle_manager]: 
[INFO] [lifecycle_manager]: Step: Configure Perception Node
[INFO] [lifecycle_manager]: Transitioning perception_node: unconfigured -> inactive
[INFO] [lifecycle_manager]: ✅ perception_node successfully transitioned to inactive
[INFO] [lifecycle_manager]: 
[INFO] [lifecycle_manager]: ============================================================
[INFO] [lifecycle_manager]: ✅ Lifecycle Management Complete - System Ready
[INFO] [lifecycle_manager]: ============================================================
[INFO] [lifecycle_manager]: Lifecycle manager will now exit (startup complete)
```

## Error Handling

### Node Not Available

If a node doesn't become available within the timeout:

```
[ERROR] [lifecycle_manager]: Timeout waiting for system_manager after 30.0s
[ERROR] [lifecycle_manager]: Failed to detect system_manager
```

**Solution**: Ensure the node is included in the launch file and starting correctly.

### Transition Failure

If a transition request fails:

```
[ERROR] [lifecycle_manager]: ❌ Failed to transition perception_node to inactive
[ERROR] [lifecycle_manager]: Startup sequence failed at: Configure Perception Node
```

**Solution**: Check the target node's logs for errors in its lifecycle callbacks.

### State Timeout

If a node accepts the transition but never reaches the target state:

```
[ERROR] [lifecycle_manager]: ❌ Timeout waiting for system_manager to reach active
```

**Solution**: The node's lifecycle callback may be stuck or returning an error. Check node logs.

## Advantages Over Timer-Based Approaches

| Aspect | Timer-Based | Lifecycle Manager |
|--------|-------------|-------------------|
| **Reliability** | Fixed delays may be too short or too long | Waits for actual state changes |
| **Performance** | Wastes time on fast systems | Proceeds as soon as ready |
| **Robustness** | Can fail on slow systems | Adapts to system performance |
| **Debugging** | Hard to diagnose timing issues | Clear state transition logs |
| **Portability** | Different timing on different machines | Works consistently across platforms |

## Integration with System Manager

The Lifecycle Manager handles **initial startup** only. Once nodes are configured:

1. **Lifecycle Manager** configures and activates `system_manager`
2. **Lifecycle Manager** configures (but doesn't activate) `perception_node`
3. **System Manager** takes over operational state management
4. **System Manager** activates/deactivates nodes based on operational states

```
Startup Phase                    Operational Phase
──────────────                   ─────────────────
Lifecycle Manager  ───────────→  System Manager
    │                                   │
    ├─ Configure nodes                  ├─ Activate on AUTONOMOUS
    ├─ Activate core nodes              ├─ Deactivate on STANDBY
    └─ Exit when complete               └─ Manage during operation
```

## Testing

See `test/test_lifecycle_manager.py` for comprehensive test coverage:

- Node detection with timeouts
- State querying and validation
- Transition execution and verification
- Error handling scenarios
- Dynamic sequence building

Run tests:
```bash
# Run all lifecycle manager tests
colcon test --packages-select grizzly_stack --pytest-args -k test_lifecycle_manager

# Run specific test
colcon test --packages-select grizzly_stack --pytest-args -k test_wait_for_node_success
```

## Future Enhancements

Potential improvements for the lifecycle manager:

1. **Parallel Transitions** - Transition independent nodes simultaneously
2. **Dependency Graph** - Define node dependencies for intelligent ordering
3. **Recovery Policies** - Automatically retry failed transitions
4. **Health Monitoring** - Track node health during transitions
5. **Configuration File** - Allow custom sequences via YAML config

## Related Documentation

- [Lifecycle Implementation Summary](LIFECYCLE_IMPLEMENTATION_SUMMARY.md)
- [Lifecycle Diagram](LIFECYCLE_DIAGRAM.md)
- [State Management Guide](STATE_MANAGEMENT_GUIDE.md)
- [System Manager](../README.md#system-manager)
