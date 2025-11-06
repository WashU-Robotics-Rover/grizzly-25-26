# Grizzly Web Interface - Quick Reference

## 🚀 Quick Start

The app automatically initializes when the page loads. Open browser console to see initialization logs.

## 📦 Module Reference

### Config (core/config.js)
```javascript
Config.states.AUTONOMOUS        // State number: 2
Config.stateNames[2]            // State name: "AUTONOMOUS"
Config.stateColors[2]           // Color scheme for state
Config.health.timeoutMs         // Health timeout: 5000ms
Config.topics.state.name        // Topic: "/system/state"
Config.services.changeState.name // Service: "/system/change_state"
```

### Utils (utils/helpers.js)
```javascript
Utils.formatROSTime(timestamp)  // Format ROS timestamp
Utils.getElement('elementId')   // Safe DOM element access
Utils.setText('id', 'text')     // Set element text
Utils.setClass('id', 'classes') // Set element classes
Utils.log('info', 'message')    // Timestamped logging
```

### ROSConnection (ros/connection.js)
```javascript
ROSConnection.connect()         // Connect to ROS Bridge
ROSConnection.reconnect()       // Reconnect
ROSConnection.getConnectionStatus() // Check if connected
ROSConnection.getRos()          // Get ROSLIB.Ros instance
ROSConnection.on('onConnect', callback) // Register callback
```

### ROSSubscribers (ros/subscribers.js)
```javascript
ROSSubscribers.setup()          // Setup all subscriptions
ROSSubscribers.cleanup()        // Cleanup subscriptions
ROSSubscribers.on('onStateUpdate', callback)  // State updates
ROSSubscribers.on('onHealthUpdate', callback) // Health updates
```

### ROSServices (ros/services.js)
```javascript
ROSServices.setup()             // Setup service clients
ROSServices.requestStateChange(state, reason) // Returns Promise
ROSServices.cleanup()           // Cleanup clients
```

### StateDisplay (ui/state-display.js)
```javascript
StateDisplay.update(stateMsg)   // Update display with state
StateDisplay.setUnknownState()  // Show UNKNOWN state
StateDisplay.getStateName(2)    // Get name for state number
```

### HealthDisplay (ui/health-display.js)
```javascript
HealthDisplay.update(healthMsg) // Update health display
HealthDisplay.setWarningState() // Show warning state
HealthDisplay.reset()           // Reset to waiting state
```

### HistoryManager (ui/history.js)
```javascript
HistoryManager.add(stateMsg)    // Add entry to history
HistoryManager.render()         // Render history list
HistoryManager.clear()          // Clear all history
HistoryManager.getHistory()     // Get history array
HistoryManager.exportJSON()     // Export as JSON string
```

### HealthMonitor (monitoring/health-monitor.js)
```javascript
HealthMonitor.start()           // Start monitoring
HealthMonitor.stop()            // Stop monitoring
HealthMonitor.reset()           // Reset timeout timer
HealthMonitor.recordUpdate()    // Record health update received
HealthMonitor.getHealthStatus() // Check if healthy
```

### Notifications (ui/notifications.js)
```javascript
Notifications.success('message')    // Show success
Notifications.error('message')      // Show error
Notifications.warning('message')    // Show warning
Notifications.info('message')       // Show info
Notifications.showServiceResponse(msg, success) // Show response
Notifications.hideServiceResponse() // Hide response
```

### GrizzlyApp (main.js)
```javascript
GrizzlyApp.init()               // Initialize app (auto-called)
GrizzlyApp.reconnect()          // Reconnect to ROS
GrizzlyApp.getStatus()          // Get app status object
```

## 🌐 Global Functions (for HTML)

```javascript
window.requestStateChange(targetState, reason)  // Change state
window.reconnect()                               // Reconnect
window.GrizzlyDebug                              // Debug object
```

## 🐛 Debug Console Examples

```javascript
// Check connection status
GrizzlyDebug.connection.getConnectionStatus()

// View all config
console.table(GrizzlyDebug.config.stateNames)

// Get current app status
GrizzlyDebug.app.getStatus()

// View history
console.table(GrizzlyDebug.history.getHistory())

// Check health
GrizzlyDebug.healthMonitor.getHealthStatus()

// Manual state display update
GrizzlyDebug.stateDisplay.update({
    state: 2,
    description: 'Test',
    timestamp: { sec: Date.now()/1000, nanosec: 0 }
})

// Trigger test notification
GrizzlyDebug.notifications.success('Test notification!')

// Export history to clipboard
copy(GrizzlyDebug.history.exportJSON())
```

## 📊 State Numbers Reference

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

## 🔌 Event Flow

### Connection Established
```
ROSConnection fires 'onConnect'
  → ROSSubscribers.setup()
  → ROSServices.setup()
  → HealthMonitor.start()
```

### State Update Received
```
ROSSubscribers fires 'onStateUpdate'
  → StateDisplay.update(message)
  → HistoryManager.add(message)
```

### Health Update Received
```
ROSSubscribers fires 'onHealthUpdate'
  → HealthDisplay.update(message)
  → HealthMonitor.recordUpdate()
```

### Health Timeout
```
HealthMonitor timeout fires
  → StateDisplay.setUnknownState()
  → HealthDisplay.setWarningState()
  → Notifications.warning(...)
```

## 🎯 Common Tasks

### Add Custom Logging
```javascript
Utils.log('info', 'My message', { data: 'optional' })
```

### Trigger Manual State Change
```javascript
window.requestStateChange(Config.states.STANDBY, 'User request')
```

### Check if Connected
```javascript
if (ROSConnection.getConnectionStatus()) {
    console.log('Connected!');
}
```

### Get Recent History
```javascript
const recent = HistoryManager.getHistory().slice(0, 5)
console.table(recent)
```

### Monitor Health Status
```javascript
setInterval(() => {
    const status = HealthMonitor.getHealthStatus()
    const lastUpdate = HealthMonitor.getLastUpdateTime()
    console.log(`Healthy: ${status}, Last update: ${lastUpdate}`)
}, 1000)
```

## 📝 Best Practices

1. **Don't hardcode values** - Use `Config` object
2. **Don't access DOM directly** - Use `Utils` helpers
3. **Don't use console.log** - Use `Utils.log()`
4. **Register callbacks** - Don't poll for changes
5. **Handle errors** - Always catch and notify user
6. **Use promises** - For async operations
7. **Log important events** - Help debugging

## 🔧 Troubleshooting

### Nothing appears on screen
- Check browser console for errors
- Verify all script tags loaded in correct order
- Check `GrizzlyDebug` object exists

### Can't connect to ROS
- Verify rosbridge_server is running: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
- Check WebSocket URL in connection status
- Look for connection errors in console

### State/Health not updating
- Check if subscriptions are active: `GrizzlyDebug.subscribers`
- Verify topics are publishing: `ros2 topic list`
- Check for subscription errors in console

### Service calls failing
- Verify service exists: `ros2 service list`
- Check service is ready in UI
- Review service call errors in console
