# 🐻 Grizzly Command - Web Interface

A modern web-based monitoring and control interface for the Grizzly Rover system using roslibjs.

**✨ Integrated with Grizzly CLI - Launch with: `./grizzly.py web`**

## Features

- 📊 **Real-time State Monitoring** - Live updates of the rover's operational state
- 💚 **Health Status Display** - Visual health indicators and status messages
- 🎮 **State Control** - Request state transitions with interactive buttons
- 📜 **State History** - Track recent state changes with timestamps
- 🔌 **ROS Bridge Connection** - Seamless integration with ROS 2 via rosbridge

## Screenshots

The interface provides:
- Current operational state display (STARTUP, STANDBY, AUTONOMOUS, MANUAL, EMERGENCY, ERROR, SHUTDOWN)
- Real-time health monitoring with visual indicators
- One-click state transition controls
- Historical state change log
- Connection status monitoring

## Prerequisites

### ROS 2 Requirements

1. **Running ROS 2 System**
   - The Grizzly stack must be running
   - System manager node must be active

2. **rosbridge_server**
   - Required to bridge ROS 2 topics/services to WebSockets
   - Install if not already present:
     ```bash
     sudo apt install ros-humble-rosbridge-server
     # or for other distributions:
     sudo apt install ros-<distro>-rosbridge-server
     ```

3. **Active Topics**
   - `/system/state` - Operational state updates
   - `/system/health` - Health status messages

4. **Active Services**
   - `/system/change_state` - State transition requests

## Installation

No installation needed! The web interface is self-contained and uses:
- roslibjs from CDN (no npm required)
- Pure HTML, CSS, and JavaScript
- No build process needed
- **Integrated into Grizzly CLI for easy launching**

## Quick Start

### The Easy Way (Recommended)

**✨ Launch everything with one command:**

```bash
# 1. Start the Grizzly System (in one terminal)
cd /Users/danielhuinda/robotics/grizzly-25-26
./grizzly.py run

# 2. Launch the web interface (in another terminal)
cd /Users/danielhuinda/robotics/grizzly-25-26
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

### Manual Method (Advanced)

If you need to run components separately:

#### 1. Launch the Grizzly System

```bash
cd /Users/danielhuinda/robotics/grizzly-25-26
./grizzly.py run
```

#### 2. Start rosbridge_server (with workspace sourced)

```bash
cd /Users/danielhuinda/robotics/grizzly-25-26
source install/setup.zsh  # Critical for grizzly_interfaces
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

#### 3. Serve the web interface

```bash
cd /Users/danielhuinda/robotics/grizzly-25-26/grizzly_command
python3 -m http.server 8000
```

#### 4. Open Browser

Navigate to: `http://localhost:8000`

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

## Configuration

### Custom ROS Bridge URL

When using `./grizzly.py web`:

```bash
# Custom ports
./grizzly.py web --port 3000 --rosbridge-port 9091
```

Or if manually running, edit the URL in the settings panel, or modify the default in `index.html`:
```html
<input type="text" id="rosBridgeUrl" value="ws://YOUR_HOST:YOUR_PORT" />
```

## Architecture

### Technology Stack

- **Frontend**: Pure HTML5, CSS3, JavaScript (ES6)
- **ROS Integration**: roslibjs (from CDN)
- **Communication**: WebSockets via rosbridge_server
- **Styling**: Custom CSS with responsive design

### ROS Integration

The interface uses roslibjs to:

1. **Connect** to rosbridge_server via WebSocket
2. **Subscribe** to topics:
   - `/system/state` (grizzly_interfaces/msg/OperationalState)
   - `/system/health` (std_msgs/msg/String)
3. **Call Services**:
   - `/system/change_state` (grizzly_interfaces/srv/ChangeState)

### Data Flow

```
ROS 2 System → rosbridge_server → WebSocket → Browser
                                      ↓
                                   roslibjs
                                      ↓
                              Web Interface UI
```

## Troubleshooting

### ⚠️ "No module named 'grizzly_interfaces'" Error

**Problem**: Service calls fail with: `Unable to import grizzly_interfaces.srv from package grizzly_interfaces`

**Solution**: This is the most common issue! rosbridge_server must be launched from a terminal that has sourced your workspace.

```bash
# In the terminal where you launch rosbridge:
cd /Users/danielhuinda/robotics/grizzly-25-26
source install/setup.zsh  # ← CRITICAL STEP!
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**Why**: rosbridge_server needs access to your custom message/service definitions. Without sourcing the workspace, it can't find `grizzly_interfaces`.

**Verify it worked**:
```bash
# After sourcing, check if interfaces are available:
ros2 interface list | grep grizzly_interfaces
# Should show grizzly_interfaces/msg/OperationalState, etc.
```

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

### CORS Issues

**Problem**: Browser blocks WebSocket connection

**Solutions**:
1. Always serve files via HTTP server (not `file://`)
2. Ensure rosbridge allows connections from your domain
3. Use `python3 -m http.server` or the provided `serve.py`

## Development

### File Structure

```
grizzly_command/
├── index.html          # Main HTML interface
├── css/
│   └── styles.css      # All styling
├── js/
│   └── app.js          # Application logic & ROS integration
├── README.md           # This file
└── PROJECT_SUMMARY.md  # Project documentation
```

**Note**: This directory is integrated with the Grizzly CLI. Use `./grizzly.py web` to launch!

### Customization

#### Adding New Topics

Edit `js/app.js` and add a new subscriber in `setupROSSubscribers()`:

```javascript
const myTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/my/topic',
    messageType: 'my_package/msg/MyMessage'
});

myTopic.subscribe(function(message) {
    console.log('Received:', message);
    // Update UI here
});
```

#### Styling Changes

All styles are in `css/styles.css`. Color scheme is defined in CSS variables:

```css
:root {
    --primary-color: #2c3e50;
    --accent-color: #3498db;
    /* ... */
}
```

#### Adding Features

The modular structure makes it easy to add:
- Additional panels in `index.html`
- New ROS service calls in `js/app.js`
- Extra visualizations with styling in `css/styles.css`

## Related Documentation

- [Grizzly System Architecture](../docs/ARCHITECTURE.md)
- [State Machine Guide](../docs/STATE_MACHINE_GUIDE.md)
- [ROS Reference](../docs/ROS_REFERENCE.md)
- [roslibjs Documentation](http://wiki.ros.org/roslibjs)
- [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)

## License

Part of the Grizzly Rover project - WashU Robotics Rover Team

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review the main Grizzly documentation
3. Contact the Grizzly development team

---

**Built with ❤️ for the Grizzly Rover Team**
