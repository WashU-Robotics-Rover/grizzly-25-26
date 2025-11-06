# Tab System Feature Documentation

## Overview

The Grizzly Command Interface now features a tabbed navigation system that organizes content into logical sections:

1. **System Management** - Original control panel with state management, health monitoring, and controls
2. **Output** - Real-time terminal display of ROS log messages (/rosout)

## Features

### Tab Navigation
- Clean, minimalist tab buttons in the header
- Smooth transitions between tabs
- Active tab indication with blue highlight
- Keyboard-accessible navigation

### System Management Tab
Contains all original features:
- Operational State panel
- Health Monitor
- ROS Bridge connection settings
- State Control buttons
- Event Log (state history)
- State Reference guide

### Output Tab
New terminal-style log viewer:
- **Real-time ROS logs** from `/rosout` topic
- **Color-coded log levels**:
  - DEBUG (blue)
  - INFO (green)
  - WARN (yellow)
  - ERROR (red)
  - FATAL (dark red)
- **Filterable output** - Toggle visibility of each log level
- **Auto-scroll** - Automatically scroll to latest messages (toggleable)
- **Statistics** - Live counters for each log level
- **Clear button** - Reset terminal display
- **Performance** - Handles up to 1000 messages with automatic pruning

## Technical Implementation

### New Modules

#### 1. `ui/tab-manager.js` - TabManager
Handles tab switching logic and UI updates.

**API:**
```javascript
TabManager.switchTab('system-management')  // Switch to System Management
TabManager.switchTab('output')             // Switch to Output
TabManager.getCurrentTab()                 // Get active tab name
```

#### 2. `ui/terminal-output.js` - TerminalOutput
Manages the terminal display and message filtering.

**API:**
```javascript
TerminalOutput.init()                      // Initialize terminal
TerminalOutput.addMessage(rosoutMsg)       // Add ROS log message
TerminalOutput.clear()                     // Clear all messages
TerminalOutput.toggleAutoscroll()          // Toggle auto-scroll
TerminalOutput.toggleLevel('INFO')         // Toggle log level filter
TerminalOutput.scrollToBottom()            // Scroll to latest
TerminalOutput.exportJSON()                // Export logs as JSON
TerminalOutput.exportText()                // Export logs as text
```

### Configuration Updates

Added to `core/config.js`:
```javascript
topics: {
    rosout: {
        name: '/rosout',
        messageType: 'rcl_interfaces/msg/Log'
    }
}
```

### Subscriber Updates

Added to `ros/subscribers.js`:
- New `rosoutListener` subscriber
- New `onRosoutUpdate` callback
- Automatic subscription on connection

### Data Flow

```
ROS /rosout topic
    ↓
ROSSubscribers._subscribeToRosout()
    ↓
ROSSubscribers fires 'onRosoutUpdate'
    ↓
main.js callback
    ↓
TerminalOutput.addMessage(message)
    ↓
Render to terminal (if level filter allows)
    ↓
Update statistics
    ↓
Auto-scroll (if enabled)
```

## Usage Examples

### Switching Tabs Programmatically
```javascript
// From HTML onclick
<button onclick="switchTab('output')">Go to Output</button>

// From JavaScript
TabManager.switchTab('output');
```

### Filtering Terminal Output
```javascript
// Hide debug messages
TerminalOutput.toggleLevel('DEBUG');

// Show only errors and fatals
['DEBUG', 'INFO', 'WARN'].forEach(level => {
    TerminalOutput.toggleLevel(level);
});
```

### Exporting Logs
```javascript
// Export to clipboard
const logs = TerminalOutput.exportText();
navigator.clipboard.writeText(logs);

// Export as JSON
const jsonLogs = TerminalOutput.exportJSON();
console.log(jsonLogs);
```

### Debugging
```javascript
// Access terminal from console
GrizzlyDebug.terminal.messages         // View all messages
GrizzlyDebug.terminal.stats            // View statistics
GrizzlyDebug.tabManager.getCurrentTab() // Check active tab
```

## ROS Message Format

The terminal expects `/rosout` messages with this structure:
```javascript
{
    header: {
        stamp: { sec: 123456, nanosec: 789000000 }
    },
    level: 2,  // 1=DEBUG, 2=INFO, 4=WARN, 8=ERROR, 16=FATAL
    name: 'node_name',
    msg: 'Log message text',
    file: 'source_file.cpp',
    function: 'function_name',
    line: 42
}
```

## Styling

### Tab Buttons
```css
.tab-button.active {
    border-bottom: 2px solid #3b82f6;
    color: #60a5fa;
}
```

### Terminal Colors
- Background: Pure black (#000000)
- Text levels: Blue (DEBUG), Green (INFO), Yellow (WARN), Red (ERROR/FATAL)
- Timestamps: Gray (#71717a)
- Node names: Medium gray (#a1a1aa)

### Custom Scrollbar
Styled thin scrollbar for better UX in terminal container.

## Performance Considerations

1. **Message Limit**: Terminal stores max 1000 messages to prevent memory issues
2. **Throttling**: Consider throttling high-frequency log sources
3. **Filtering**: Disabled filters prevent rendering, improving performance
4. **Auto-scroll**: Only scrolls when visible and enabled

## Future Enhancements

Potential additions to the Output tab:
- [ ] Search/filter by node name
- [ ] Search/filter by message text (regex support)
- [ ] Export to file download
- [ ] Color themes
- [ ] Font size controls
- [ ] Pause/resume output
- [ ] Timestamp format options
- [ ] Message grouping/collapsing
- [ ] Line numbers
- [ ] Copy individual messages
- [ ] Link detection in messages

## Browser Compatibility

Tested and working on:
- Chrome/Edge 90+
- Firefox 88+
- Safari 14+

## Accessibility

- Keyboard navigation supported
- ARIA labels on interactive elements
- High contrast color choices
- Readable font sizes

## CSS Classes Reference

```css
.tab-button          /* Tab button styling */
.tab-button.active   /* Active tab highlight */
.tab-content         /* Tab content container */
.terminal-line       /* Individual log line */
.filter-button       /* Log level filter button */
.filter-button.active /* Active filter state */
```

## Global Functions

Exposed to `window` for HTML onclick handlers:
```javascript
window.switchTab(tabName)                    // Switch tabs
window.TerminalOutput.clear()                // Clear terminal
window.TerminalOutput.toggleAutoscroll()     // Toggle autoscroll
window.TerminalOutput.toggleLevel(level)     // Toggle filter
```
