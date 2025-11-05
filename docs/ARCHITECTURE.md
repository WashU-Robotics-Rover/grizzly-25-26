# System Architecture

Comprehensive overview of the Grizzly Rover software architecture, component interactions, and design decisions.

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [System Components](#system-components)
- [Data Flow](#data-flow)
- [State Management Architecture](#state-management-architecture)
- [Layer-Based Organization](#layer-based-organization)
- [Design Patterns](#design-patterns)
- [Communication Patterns](#communication-patterns)

## Architecture Overview

The Grizzly Rover software stack is built on ROS 2 with a modular, lifecycle-managed architecture designed for autonomous rover operations. The system uses a layered approach to manage complexity and scalability.

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Grizzly Rover Stack                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                    Core Layer                           │  │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │  │
│  │  │   System     │  │  Lifecycle   │  │   Layer      │   │  │
│  │  │   Manager    │→ │   Manager    │→ │   Manager    │   │  │
│  │  └──────────────┘  └──────────────┘  └──────────────┘   │  │
│  └──────────────────────────────────────────────────────────┘  │
│                            │                                     │
│        ┌───────────────────┼───────────────────┐                 │
│        ↓                   ↓                   ↓                 │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│  │  Perception  │  │   Planning   │  │   Control    │         │
│  │   Layer      │→ │    Layer     │→ │   Layer      │         │
│  └──────────────┘  └──────────────┘  └──────────────┘         │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Key Design Principles

1. **Lifecycle Management**: All nodes use ROS 2 Lifecycle for deterministic startup/shutdown
2. **Layer-Based Organization**: Nodes grouped into logical layers for coordinated management
3. **State-Driven Coordination**: Operational states drive system behavior
4. **Modularity**: Clear separation between subsystems (perception, planning, control)
5. **Scalability**: Easy to add new nodes without modifying core management code

## System Components

### Core Layer

The core layer provides system-wide coordination and management.

#### System Manager

**Purpose**: Central orchestration of operational states

**Responsibilities:**
- Manage operational state machine (STARTUP, STANDBY, AUTONOMOUS, MANUAL, EMERGENCY, ERROR, SHUTDOWN)
- Publish system state and health status
- Coordinate state transitions
- Delegate layer activation to Layer Manager

**Interfaces:**
- **Published Topics**: `/system/state`, `/system/health`
- **Services**: `/system/change_state`
- **Lifecycle**: Manages its own lifecycle transitions

**State Transitions:**
```
STARTUP → STANDBY → AUTONOMOUS/MANUAL
                    ↓
               EMERGENCY/ERROR → SHUTDOWN
```

---

#### Layer Manager

**Purpose**: Manage nodes organized into logical layers

**Responsibilities:**
- Activate/deactivate entire layers based on operational state
- Coordinate lifecycle transitions for layer nodes
- Map operational states to layer activation
- Handle async lifecycle transitions

**Configuration:**
- Defined in `grizzly_stack/config/layers.yaml`
- Layer definitions include node lists and startup order

**Layer-State Mapping:**
- **AUTONOMOUS/MANUAL**: Activates perception, planning, control layers
- **STANDBY/STARTUP**: No operational layers active
- **EMERGENCY/ERROR/SHUTDOWN**: Deactivates all layers immediately

---

#### Lifecycle Manager

**Purpose**: Orchestrate startup of all lifecycle-managed nodes

**Responsibilities:**
- Detect node availability before transitions
- Configure nodes in proper startup order
- Verify state transitions
- Exit cleanly after startup complete

**Startup Sequence:**
1. Wait for lifecycle services to become available
2. Configure nodes by layer (respecting startup_order)
3. Activate system_manager
4. Exit (system_manager takes over)

**Design Philosophy:**
- Event-based rather than timer-based
- Adapts to system performance
- Platform-independent (works on macOS, Linux, Windows)

---

### Perception Layer

**Purpose**: Sensor processing and environmental awareness

**Components:**
- `perception_node`: Template perception node
- `central_perception`: Central sensor processing

**Data Flow:**
```
Sensors → Perception Nodes → PerceptionState → Planning Layer
```

**Key Features:**
- Lifecycle-managed nodes
- State estimation (pose, velocity, terrain)
- Configurable processing rates
- Frame-aware (base_link, map, odom)

**Active When:**
- AUTONOMOUS mode (always)
- MANUAL mode (optional, configurable)

---

### Planning Layer

**Purpose**: High-level decision making and path planning

**Components:**
- `planner_node`: Template planner
- `drive_planner`: Drive system planning
- `arm_planner`: Arm manipulation planning
- `science_planner`: Science operations planning

**Data Flow:**
```
PerceptionState → Planning Nodes → Commands → Control Layer
```

**Key Features:**
- Multi-subsystem planning (drive, arm, science)
- State-aware planning (considers operational state)
- Path and behavior planning

**Active When:**
- AUTONOMOUS mode
- MANUAL mode

---

### Control Layer

**Purpose**: Low-level motor and actuator control

**Components:**
- `control_node`: Template control node

**Data Flow:**
```
Planning Commands → Control Nodes → Hardware Actuators
```

**Key Features:**
- Command execution
- Feedback publishing
- Safety monitoring

**Active When:**
- AUTONOMOUS mode
- MANUAL mode

---

## Data Flow

### Startup Sequence

```
1. Launch File
   ↓
2. Lifecycle Manager
   ├─ Detect node availability
   ├─ Configure nodes by layer
   └─ Activate system_manager
   ↓
3. System Manager (Active)
   ├─ Initial state: STARTUP
   ├─ Transition to STANDBY
   └─ Ready for operational commands
   ↓
4. Operational Phase
   ├─ State transitions via service calls
   ├─ Layer Manager activates/deactivates layers
   └─ Nodes process data and execute commands
```

### Operational Data Flow

```
┌──────────┐      ┌──────────────┐      ┌──────────┐      ┌──────────┐
│ Sensors  │ ───→ │ Perception   │ ───→ │ Planning │ ───→ │ Control  │
│          │      │  Layer       │      │  Layer   │      │  Layer   │
└──────────┘      └──────────────┘      └──────────┘      └──────────┘
                        │                      │                  │
                        └──────────────────────┴──────────────────┘
                                         │
                                         ↓
                                   ┌──────────┐
                                   │ System   │
                                   │ Manager  │
                                   └──────────┘
```

### State Coordination Flow

```
External Request → System Manager → Operational State Change
                                      │
                                      ↓
                              Layer Manager
                                      │
                ┌────────────────────┼────────────────────┐
                ↓                    ↓                    ↓
         Perception Layer    Planning Layer      Control Layer
                │                    │                    │
                └────────────────────┴────────────────────┘
                                      │
                                      ↓
                              Layer Activation/
                              Deactivation
```

## State Management Architecture

### Two-Layer State System

The system uses two complementary state management layers:

#### 1. Operational States (Mission Level)

**Managed By**: System Manager  
**Purpose**: Mission-level coordination

```
STARTUP → STANDBY ⇄ AUTONOMOUS ⇄ MANUAL
              ↓
      EMERGENCY / ERROR / SHUTDOWN
```

**Characteristics:**
- Mission-level semantics
- User-visible states
- Coordinated across entire system
- Drives layer activation

#### 2. Lifecycle States (Node Level)

**Managed By**: ROS 2 Lifecycle + Layer Manager  
**Purpose**: Node resource management

```
Unconfigured → Inactive → Active → Finalized
```

**Characteristics:**
- Resource management (memory, timers, publishers)
- Deterministic startup/shutdown
- Platform-independent
- ROS 2 standard

### State Interaction

```
Operational State Change
         │
         ↓
System Manager evaluates transition
         │
         ├─ Valid → Update operational state
         │              │
         │              ↓
         │         Layer Manager
         │              │
         │              ├─ Determine affected layers
         │              │
         │              ↓
         │         Lifecycle Transitions
         │              │
         │              ├─ Activate/Deactivate layers
         │              │
         │              ↓
         │         Nodes change lifecycle state
         │              │
         │              ↓
         └──────── Node resources managed
```

## Layer-Based Organization

### Why Layers?

Traditional approaches manage nodes individually, which leads to:
- System Manager bloat as nodes are added
- Complex state coordination logic
- Difficult to maintain and extend

**Layer-based approach:**
- Groups related nodes together
- Manages layers as units
- Scales easily (add nodes to layers)
- Configuration-driven (no code changes needed)

### Layer Structure

```
layers:
  perception:
    nodes: [perception_node]
    startup_order: 1
    
  planning:
    nodes: [planner_node]
    startup_order: 2
    
  control:
    nodes: [control_node]
    startup_order: 3
```

### Adding New Nodes

To add a new node:

1. **Add to existing layer** (if it fits):
```yaml
layers:
  perception:
    nodes:
      - perception_node
      - new_perception_node  # Add here
```

2. **Create new layer** (if needed):
```yaml
layers:
  navigation:
    nodes:
      - navigation_node
    startup_order: 2
```

No code changes to System Manager or Layer Manager required!

## Design Patterns

### Lifecycle Pattern

All operational nodes implement ROS 2 Lifecycle:

```python
class MyNode(LifecycleNode):
    def on_configure(self, state):
        # Allocate resources
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state):
        # Start processing
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state):
        # Stop processing
        return TransitionCallbackReturn.SUCCESS
```

### State-Driven Pattern

Nodes react to operational state changes:

```python
def operational_state_callback(self, msg):
    if msg.state == OperationalState.AUTONOMOUS:
        # Enable autonomous behavior
    elif msg.state == OperationalState.STANDBY:
        # Pause processing
```

### Layer Coordination Pattern

Layer Manager coordinates node lifecycle based on operational state:

```python
# Pseudo-code
if operational_state == AUTONOMOUS:
    activate_layer('perception')
    activate_layer('planning')
    activate_layer('control')
elif operational_state == STANDBY:
    deactivate_all_layers()
```

## Communication Patterns

### Topic-Based Communication

**Use Case**: Continuous data streams (sensor data, state updates)

**Examples:**
- `/system/state` - Operational state updates
- `/perception/state` - Perception data
- Sensor topics

**Characteristics:**
- Publisher/subscriber pattern
- Fire-and-forget
- Best for high-frequency data
- Decoupled (publisher doesn't know subscribers)

### Service-Based Communication

**Use Case**: Request-response operations

**Examples:**
- `/system/change_state` - State transition requests
- Lifecycle service calls

**Characteristics:**
- Synchronous or asynchronous
- Request-response pattern
- Best for infrequent operations
- Coupled (client knows server)

### Lifecycle Service Pattern

**Use Case**: Node lifecycle management

**Pattern:**
```python
# Lifecycle Manager calls lifecycle services
lifecycle_client.call_async(configure_request)
lifecycle_client.call_async(activate_request)
```

**Used By:**
- Lifecycle Manager (startup)
- Layer Manager (operational phase)
- System Manager (self-management)

## Component Interactions

### Startup Interaction

```
Launch File
    │
    ├─→ Lifecycle Manager
    │       │
    │       ├─→ Configure system_manager
    │       ├─→ Configure perception nodes
    │       ├─→ Configure planning nodes
    │       ├─→ Configure control nodes
    │       │
    │       └─→ Activate system_manager
    │               │
    │               └─→ System Manager (Active)
    │                       │
    │                       └─→ Initial state: STARTUP
    │
    └─→ Lifecycle Manager exits
```

### Operational Interaction

```
State Change Request
    │
    ↓
System Manager
    │
    ├─→ Validate transition
    ├─→ Update operational state
    │
    └─→ Layer Manager
            │
            ├─→ Determine affected layers
            ├─→ Activate/Deactivate layers
            │
            └─→ Lifecycle transitions
                    │
                    ├─→ Perception nodes
                    ├─→ Planning nodes
                    └─→ Control nodes
```

### Data Processing Interaction

```
Sensors
    │
    ↓
Perception Layer (Active)
    │
    ├─→ Process sensor data
    ├─→ Estimate state
    │
    └─→ Publish PerceptionState
            │
            ↓
Planning Layer (Active)
    │
    ├─→ Receive PerceptionState
    ├─→ Plan actions
    │
    └─→ Publish commands
            │
            ↓
Control Layer (Active)
    │
    ├─→ Receive commands
    ├─→ Execute on hardware
    │
    └─→ Publish feedback
```

## Configuration Management

### Configuration Files

- **`core.yaml`**: System parameters (health rates, etc.)
- **`layers.yaml`**: Layer definitions and node organization

### Configuration Philosophy

- **YAML-based**: Human-readable, version-controlled
- **No code changes**: Modify config files and relaunch
- **Layer-driven**: Nodes organized by configuration
- **Parameter-driven**: Node behavior controlled by parameters

## Extensibility

### Adding New Subsystems

1. **Create new layer** in `layers.yaml`
2. **Implement nodes** following lifecycle pattern
3. **Define interfaces** in `grizzly_interfaces` if needed
4. **Add to launch file** if standalone launch needed
5. **No core changes** required

### Adding New Nodes to Existing Layers

1. **Add node name** to layer in `layers.yaml`
2. **Implement node** following lifecycle pattern
3. **Test with existing layer**
4. **No core changes** required

## See Also

- [State Management Guide](STATE_MANAGEMENT_GUIDE.md) - Detailed state management
- [ROS Reference](ROS_REFERENCE.md) - Complete ROS 2 nodes, topics, services, and messages reference
- [Lifecycle Manager](LIFECYCLE_MANAGER.md) - Lifecycle orchestration details
- [Development Guide](DEVELOPMENT.md) - Development practices

