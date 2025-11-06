# Grizzly Web Interface - Modular Architecture

## 📁 Directory Structure

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
│   │   └── history.js          # History management
│   ├── monitoring/
│   │   └── health-monitor.js   # Health timeout detection
│   ├── utils/
│   │   └── helpers.js          # Utility functions
│   ├── main.js                 # Application orchestrator
│   └── app.js.backup           # Original monolithic file (backup)
├── css/
│   └── styles.css
└── index.html
```

## 🎯 Module Overview

### Core Modules

#### `config.js`
- Centralized configuration
- State definitions and mappings
- Color schemes
- ROS topic/service definitions
- Timeout settings
- All modules reference this for consistency

### ROS Interface

#### `connection.js` - ROSConnection
- Manages WebSocket connection to rosbridge
- Event-driven architecture (onConnect, onDisconnect, onError)
- Connection status tracking
- UI status updates

#### `subscribers.js` - ROSSubscribers
- Manages topic subscriptions
- Handles state and health topics
- Callback registration system
- Cleanup on disconnect

#### `services.js` - ROSServices
- Manages ROS service clients
- State change service calls
- Promise-based API
- Error handling

### UI Modules

#### `notifications.js` - Notifications
- Centralized notification system
- Service response messages
- Auto-hide functionality
- Type-based notifications (success, error, warning, info)

#### `state-display.js` - StateDisplay
- Operational state visualization
- Dynamic color theming
- State metadata display
- UNKNOWN state handling

#### `health-display.js` - HealthDisplay
- System health visualization
- Health indicator updates
- Warning state display
- Message-based status coloring

#### `history.js` - HistoryManager
- State change history tracking
- Limited history size (configurable)
- Render management
- Export functionality

### Monitoring

#### `health-monitor.js` - HealthMonitor
- Health timeout detection
- Automatic warning triggers
- Timer management
- Status tracking

### Utilities

#### `helpers.js` - Utils
- DOM manipulation helpers
- Time formatting
- Logging utilities
- Debounce/throttle functions

### Main Application

#### `main.js` - GrizzlyApp
- Application initialization
- Module orchestration
- Event handler setup
- Global function exposure
- Debug interface

## 🔄 Data Flow

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
   ↓
6. User interactions:
   - Button click → ROSServices.requestStateChange()
   - Service response → Notifications.show()
```

## 🎨 Benefits of Modular Architecture

### 1. **Separation of Concerns**
- Each module has a single, well-defined responsibility
- Easier to understand and maintain
- Changes in one area don't affect others

### 2. **Reusability**
- Modules can be used independently
- Easy to extract for other projects
- Consistent patterns across codebase

### 3. **Testability**
- Each module can be tested in isolation
- Mock dependencies easily
- Clear input/output contracts

### 4. **Scalability**
- Easy to add new features:
  - New UI panel? Add to `ui/`
  - New ROS topic? Extend `subscribers.js`
  - New service? Extend `services.js`
- Clear patterns to follow

### 5. **Debugging**
- `window.GrizzlyDebug` exposes all modules
- Targeted logging per module
- Easy to trace issues

### 6. **Configuration Management**
- All constants in one place (`config.js`)
- Easy to adjust without hunting through code
- Type safety through consistent reference

## 🚀 Adding New Features

### Add a New ROS Topic

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

### Add a New Service

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

### Add a New UI Panel

1. Create module in `ui/your-panel.js`
2. Add script tag to `index.html` (before `main.js`)
3. Use existing patterns from other UI modules

## 🐛 Debugging

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

// Manually trigger notifications
GrizzlyDebug.notifications.success('Test message')
```

## 📝 Code Conventions

1. **Naming**: Use descriptive names (no single letters except loops)
2. **Comments**: Document public methods with JSDoc style
3. **Logging**: Use `Utils.log()` for consistent logging
4. **Errors**: Handle errors gracefully, notify user
5. **Private methods**: Prefix with `_` (e.g., `_updateUI()`)
6. **Constants**: Use Config object, not hardcoded values

## 🔧 Migration Notes

The original `app.js` has been backed up as `app.js.backup`. All functionality has been preserved and reorganized into modules. The new architecture:

- ✅ Maintains all existing features
- ✅ Fixes reconnection color scheme bug
- ✅ Improves code organization
- ✅ Makes future development easier
- ✅ Adds debug capabilities
- ✅ Better error handling

## 📚 Next Steps

Consider adding:
- [ ] Unit tests for each module
- [ ] TypeScript definitions
- [ ] Additional telemetry displays
- [ ] User preferences/settings
- [ ] Command history
- [ ] Real-time graphs/charts
- [ ] Camera feed integration
- [ ] Map/navigation display
- [ ] System diagnostics panel
- [ ] Log viewer
