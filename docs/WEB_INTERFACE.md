# Grizzly Web Interface

A modern web-based monitoring and control interface for the Grizzly Rover system using roslibjs.

**✨ Integrated with Grizzly CLI - Launch with: `./grizzly.py web`**

## Features

- 📊 **Real-time State Monitoring** - Live updates of the rover's operational state
- 💚 **Health Status Display** - Visual health indicators and status messages
- 🎮 **State Control** - Request state transitions with interactive buttons
- 📜 **State History** - Track recent state changes with timestamps
- 🔌 **ROS Bridge Connection** - Seamless integration with ROS 2 via rosbridge
- 📺 **Terminal Output** - Real-time ROS log viewer with filtering
- 🏷️ **Node Filtering** - Filter logs by originating node name

## Quick Start

### The Easy Way (Recommended)

**✨ Launch everything with one command:**

```bash
# 1. Start the Grizzly System (in one terminal)
./grizzly.py run

# 2. Launch the web interface (in another terminal)
./grizzly.py web
```

That's it! The `./grizzly.py web` command will:
- ✅ Automatically start rosbridge_server (with workspace sourced)
- ✅ Launch the web server on port 8000
- ✅ Open your browser to http://localhost:8000

### Custom Options

```bash
# Use a different web port
./grizzly.py web --port 3000

# Use a different rosbridge port
./grizzly.py web --rosbridge-port 9091

# Launch only web server (if rosbridge is already running)
./grizzly.py web --no-rosbridge

# Don't open browser automatically
./grizzly.py web --no-browser
```

## Architecture

### Modular JavaScript Architecture

The web interface uses a modular architecture for maintainability and scalability:

```
grizzly_command/
├── js/
│   ├── core/
│   │   └── config.js           # Configuration and constants
│   ├── ros/
│   │   ├── connection.js       # ROS connection management
│   │   ├── subscribers.js      # Topic subscriptions
│   │   └── services.js         # Service clients
│   ├── ui/
│   │   ├── notifications.js    # Notification system
│   │   ├── state-display.js    # State visualization
│   │   ├── health-display.js   # Health visualization
│   │   ├── history.js          # History management
│   │   ├── tab-manager.js      # Tab navigation
│   │   └── terminal-output.js  # Terminal log display
│   ├── monitoring/
│   │   └── health-monitor.js   # Health timeout detection
│   ├── utils/
│   │   └── helpers.js          # Utility functions
│   └── main.js                 # Application orchestrator
├── css/
│   └── styles.css
└── index.html
```

### Module Overview

#### Core Modules

**`config.js`**
- Centralized configuration
- State definitions and mappings
- Color schemes
- ROS topic/service definitions
- Timeout settings

#### ROS Interface

**`connection.js` - ROSConnection**
- Manages WebSocket connection to rosbridge
- Event-driven architecture (onConnect, onDisconnect, onError)
- Connection status tracking
- UI status updates

**`subscribers.js` - ROSSubscribers**
- Manages topic subscriptions
- Handles state, health, and rosout topics
- Callback registration system
- Cleanup on disconnect

**`services.js` - ROSServices**
- Manages ROS service clients
- State change service calls
- Promise-based API
- Error handling

#### UI Modules

**`notifications.js` - Notifications**
- Centralized notification system
- Service response messages
- Auto-hide functionality
- Type-based notifications (success, error, warning, info)

**`state-display.js` - StateDisplay**
- Operational state visualization
- Dynamic color theming
- State metadata display
- UNKNOWN state handling

**`health-display.js` - HealthDisplay**
- System health visualization
- Health indicator updates
- Warning state display
- Message-based status coloring

**`history.js` - HistoryManager**
- State change history tracking
- Limited history size (configurable)
- Render management
- Export functionality

**`tab-manager.js` - TabManager**
- Tab switching logic
- UI updates for tab navigation
- Keyboard-accessible navigation

**`terminal-output.js` - TerminalOutput**
- Real-time ROS log display
- Color-coded log levels
- Filterable output by level and node
- Auto-scroll functionality
- Statistics tracking

#### Monitoring

**`health-monitor.js` - HealthMonitor**
- Health timeout detection
- Automatic warning triggers
- Timer management
- Status tracking

#### Utilities

**`helpers.js` - Utils**
- DOM manipulation helpers
- Time formatting
- Logging utilities
- Debounce/throttle functions

### Data Flow

```
1. User opens page
   ↓
2. main.js initializes GrizzlyApp
   ↓
3. ROSConnection.connect() establishes WebSocket
   ↓
4. On connection:
   - ROSSubscribers.setup() subscribes to topics
   - ROSServices.setup() initializes service clients
   - HealthMonitor.start() begins monitoring
   ↓
5. Topic messages arrive:
   - State updates → StateDisplay.update() + HistoryManager.add()
   - Health updates → HealthDisplay.update() + HealthMonitor.recordUpdate()
   - Rosout updates → TerminalOutput.addMessage()
   ↓
6. User interactions:
   - Button click → ROSServices.requestStateChange()
   - Service response → Notifications.show()
```

### Benefits of Modular Architecture

1. **Separation of Concerns** - Each module has a single, well-defined responsibility
2. **Reusability** - Modules can be used independently
3. **Testability** - Each module can be tested in isolation
4. **Scalability** - Easy to add new features following clear patterns
5. **Debugging** - `window.GrizzlyDebug` exposes all modules for debugging
6. **Configuration Management** - All constants in one place (`config.js`)

## Tab System

The interface features a tabbed navigation system:

### System Management Tab
- Operational State panel
- Health Monitor
- ROS Bridge connection settings
- State Control buttons
- Event Log (state history)
- State Reference guide

### Output Tab
- **Real-time ROS logs** from `/rosout` topic
- **Color-coded log levels**:
  - DEBUG (blue)
  - INFO (green)
  - WARN (yellow)
  - ERROR (red)
  - FATAL (dark red)
- **Filterable output** - Toggle visibility of each log level
- **Node filtering** - Filter messages by originating node name
- **Auto-scroll** - Automatically scroll to latest messages (toggleable)
- **Statistics** - Live counters for each log level and unique nodes
- **Clear button** - Reset terminal display
- **Performance** - Handles up to 1000 messages with automatic pruning

## ROS Integration

### Topics Subscribed

- `/system/state` (grizzly_interfaces/msg/OperationalState) - Operational state updates
- `/system/health` (std_msgs/msg/String) - Health status messages
- `/rosout` (rcl_interfaces/msg/Log) - ROS log messages

### Services Called

- `/system/change_state` (grizzly_interfaces/srv/ChangeState) - State transition requests

### Communication

The interface uses roslibjs to:
1. **Connect** to rosbridge_server via WebSocket
2. **Subscribe** to topics for real-time updates
3. **Call Services** for state transitions

## Usage

### Connection

1. The interface automatically connects to `ws://localhost:9090` on load
2. If you need to change the URL, use the settings panel
3. Click "Reconnect" to re-establish connection

### Monitoring State

The **Operational State** panel shows:
- Current state with color-coded badge
- State description
- Last update timestamp
- Numeric state value

### Monitoring Health

The **System Health** panel displays:
- Visual health indicator
- Health status message
- Last update time
- Color changes based on health (green=good, yellow=warning, red=error)

### Controlling State

Use the **State Controls** panel to request transitions:
- **STANDBY** - Put system in standby mode
- **AUTONOMOUS** - Activate autonomous operation
- **MANUAL** - Switch to manual control
- **EMERGENCY** - Trigger emergency stop

Buttons will call the `/system/change_state` service with the requested state.

### Viewing History

The **State History** panel shows:
- Recent state changes (up to 20 entries)
- Transition descriptions
- Timestamps for each change

### Terminal Output

The **Output** tab provides:
- Real-time log messages from all ROS nodes
- Filter by log level (DEBUG, INFO, WARN, ERROR, FATAL)
- Filter by node name
- Statistics showing message counts and node counts
- Export functionality (JSON and text formats)

## Development

### Adding New Features

#### Add a New ROS Topic

1. Add topic definition to `core/config.js`:
```javascript
topics: {
    // ... existing topics
    telemetry: {
        name: '/system/telemetry',
        messageType: 'grizzly_interfaces/msg/Telemetry'
    }
}
```

2. Add subscription in `ros/subscribers.js`:
```javascript
_subscribeToTelemetry() {
    this.telemetryListener = new ROSLIB.Topic({
        ros: ROSConnection.getRos(),
        name: Config.topics.telemetry.name,
        messageType: Config.topics.telemetry.messageType
    });
    
    this.telemetryListener.subscribe((message) => {
        this._triggerCallbacks('onTelemetryUpdate', message);
    });
}
```

3. Create UI module `ui/telemetry-display.js`

4. Wire it up in `main.js`:
```javascript
ROSSubscribers.on('onTelemetryUpdate', (message) => {
    TelemetryDisplay.update(message);
});
```

#### Add a New Service

1. Add to `core/config.js`:
```javascript
services: {
    // ... existing services
    resetSystem: {
        name: '/system/reset',
        serviceType: 'std_srvs/srv/Trigger'
    }
}
```

2. Add method to `ros/services.js`:
```javascript
resetSystem() {
    return new Promise((resolve, reject) => {
        // Implementation
    });
}
```

### Debugging

Access debug interface in browser console:
```javascript
// Check application status
GrizzlyDebug.connection.getConnectionStatus()

// View config
GrizzlyDebug.config

// Get history
GrizzlyDebug.history.getHistory()

// Check health status
GrizzlyDebug.healthMonitor.getHealthStatus()

// View terminal messages
GrizzlyDebug.terminal.messages

// Manually trigger notifications
GrizzlyDebug.notifications.success('Test message')
```

### Code Conventions

1. **Naming**: Use descriptive names (no single letters except loops)
2. **Comments**: Document public methods with JSDoc style
3. **Logging**: Use `Utils.log()` for consistent logging
4. **Errors**: Handle errors gracefully, notify user
5. **Private methods**: Prefix with `_` (e.g., `_updateUI()`)
6. **Constants**: Use Config object, not hardcoded values

## Troubleshooting

### ⚠️ "No module named 'grizzly_interfaces'" Error

**Problem**: Service calls fail with: `Unable to import grizzly_interfaces.srv from package grizzly_interfaces`

**Solution**: rosbridge_server must be launched from a terminal that has sourced your workspace.

```bash
# In the terminal where you launch rosbridge:
cd /Users/danielhuinda/robotics/grizzly-25-26
source install/setup.zsh  # ← CRITICAL STEP!
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**Why**: rosbridge_server needs access to your custom message/service definitions. Without sourcing the workspace, it can't find `grizzly_interfaces`.

### Cannot Connect to ROS Bridge

**Problem**: "Error connecting to ROS Bridge" message

**Solutions**:
1. Verify rosbridge is running:
   ```bash
   ros2 node list | grep rosbridge
   ```
2. Check if port 9090 is in use:
   ```bash
   lsof -i :9090
   ```
3. Verify ROS 2 system is running:
   ```bash
   ros2 topic list
   ```
4. **Make sure you sourced the workspace before launching rosbridge!**

### No State Updates

**Problem**: State panel shows "Waiting for data..."

**Solutions**:
1. Check if system_manager is publishing:
   ```bash
   ros2 topic echo /system/state
   ```
2. Verify topic type matches:
   ```bash
   ros2 topic info /system/state
   ```
3. Check browser console for JavaScript errors (F12)

### State Change Requests Fail

**Problem**: Service calls return error

**Solutions**:
1. Verify service exists:
   ```bash
   ros2 service list | grep change_state
   ```
2. Test service manually:
   ```bash
   ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
     "{requested_state: 1, reason: 'Test'}"
   ```
3. Check if current state allows transition (see state machine docs)

## Quick Reference

### Module API

#### Config (core/config.js)
```javascript
Config.states.AUTONOMOUS        // State number: 2
Config.stateNames[2]            // State name: "AUTONOMOUS"
Config.stateColors[2]           // Color scheme for state
Config.health.timeoutMs         // Health timeout: 5000ms
Config.topics.state.name        // Topic: "/system/state"
Config.services.changeState.name // Service: "/system/change_state"
```

#### ROSConnection (ros/connection.js)
```javascript
ROSConnection.connect()         // Connect to ROS Bridge
ROSConnection.reconnect()       // Reconnect
ROSConnection.getConnectionStatus() // Check if connected
ROSConnection.getRos()          // Get ROSLIB.Ros instance
```

#### ROSServices (ros/services.js)
```javascript
ROSServices.requestStateChange(state, reason) // Returns Promise
```

#### StateDisplay (ui/state-display.js)
```javascript
StateDisplay.update(stateMsg)   // Update display with state
StateDisplay.setUnknownState()  // Show UNKNOWN state
```

#### TerminalOutput (ui/terminal-output.js)
```javascript
TerminalOutput.addMessage(rosoutMsg)       // Add ROS log message
TerminalOutput.clear()                     // Clear all messages
TerminalOutput.toggleAutoscroll()          // Toggle auto-scroll
TerminalOutput.toggleLevel('INFO')         // Toggle log level filter
TerminalOutput.setNodeFilter('node_name')  // Filter by node name
```

### State Numbers Reference

```
0 - STARTUP      (Gray)
1 - STANDBY      (Blue)
2 - AUTONOMOUS   (Green)
3 - MANUAL       (Yellow)
4 - EMERGENCY    (Red)
5 - ERROR        (Dark Red)
6 - SHUTDOWN     (Black)
99 - UNKNOWN     (Yellow border, pulsing)
```

## Related Documentation

- [Grizzly System Architecture](ARCHITECTURE.md)
- [State Machine Guide](STATE_MACHINE_GUIDE.md)
- [State Management Guide](STATE_MANAGEMENT_GUIDE.md)
- [ROS Reference](ROS_REFERENCE.md)
- [roslibjs Documentation](http://wiki.ros.org/roslibjs)
- [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)

