# Terminal Node Filtering Feature

## Overview
Added node filtering capability to the Output terminal tab, allowing users to filter ROS log messages by the originating node name.

## Features Added

### 1. **Node Filter Dropdown**
- **Location**: Output Filters panel, below log level filters
- **Functionality**: Dynamically populated dropdown showing all nodes that have published messages
- **Default**: "ALL NODES" - shows messages from all nodes
- **Auto-updates**: New nodes automatically added to dropdown as they appear

### 2. **Compact Terminal Display**
- **Reduced line spacing**: Changed from `leading-relaxed` to `leading-tight py-0.5`
- **Reduced container padding**: Changed from `p-4` to `p-3`
- **Result**: More messages visible in the same screen space

### 3. **Node Statistics**
- Added "NODES" counter showing number of unique nodes
- Displayed alongside message count in terminal stats bar

## Technical Implementation

### Data Structure Changes

**TerminalOutput module** (`ui/terminal-output.js`):
```javascript
{
    nodes: new Set(),              // Tracks all unique node names
    activeNodeFilter: null,        // Current filter (null = show all)
}
```

### New Methods

#### `setNodeFilter(nodeName)`
Sets the active node filter and re-renders terminal output.
```javascript
TerminalOutput.setNodeFilter('lifecycle_manager')  // Filter to specific node
TerminalOutput.setNodeFilter(null)                 // Show all nodes
```

#### `_shouldRenderMessage(message)` (private)
Checks if a message passes all active filters (level + node).

#### `_updateNodeFilter()` (private)
Rebuilds the node filter dropdown with all discovered nodes.

### Modified Methods

#### `addMessage(message)`
- Now tracks node names in the `nodes` Set
- Calls `_updateNodeFilter()` when new nodes are discovered
- Uses `_shouldRenderMessage()` to check filters before rendering

#### `clear()`
- Resets `nodes` Set
- Resets `activeNodeFilter` to null
- Rebuilds node filter dropdown

#### `_rerenderAll()`
- Now uses `_shouldRenderMessage()` to respect node filter

### HTML Changes

**Filter Controls Section**:
```html
<select id="nodeFilterSelect" 
        onchange="TerminalOutput.setNodeFilter(this.value || null)"
        class="...">
    <option value="">ALL NODES</option>
</select>
```

**Terminal Display**:
- Each message `div` now has `data-node` attribute for easier debugging

## Usage Examples

### From Browser Console
```javascript
// Filter to specific node
TerminalOutput.setNodeFilter('lifecycle_manager')

// Show all nodes
TerminalOutput.setNodeFilter(null)

// Get list of all nodes
console.log(Array.from(TerminalOutput.nodes))

// Check current filter
console.log(TerminalOutput.activeNodeFilter)
```

### From UI
1. Click the **Output** tab
2. Scroll to **Output Filters** panel
3. Under "Node Filter", select a node from the dropdown
4. Terminal will update to show only messages from that node
5. Select "ALL NODES" to clear the filter

## Filter Combination

Filters work together:
- **Level Filter** + **Node Filter** = Show only matching level AND node
- Example: Filter to INFO messages from `lifecycle_manager` node

## Performance Considerations

1. **Node Set**: Uses JavaScript `Set` for O(1) lookup performance
2. **Efficient Re-render**: Only re-renders when filter changes, not on every message
3. **Dropdown Updates**: Only updates when new nodes appear
4. **No Throttling**: Node discovery happens in real-time

## UI/UX Improvements

### Visual Hierarchy
```
Output Filters Panel
├── Log Levels (5 buttons in grid)
└── Node Filter (dropdown below)
```

### Responsive Design
- Log level buttons: `grid-cols-2 md:grid-cols-5` (2 columns on mobile, 5 on desktop)
- Node filter: Full width for better touch targets

### Typography
- Labels: 10px uppercase mono font for consistency
- Dropdown: 12px mono font for readability
- Matches existing design language

## Statistics Display

Updated stats bar format:
```
MESSAGES: 1234  NODES: 15  |  DEBUG: 50  INFO: 1000  WARN: 150  ERROR: 30  FATAL: 4
```

## Future Enhancements

Potential additions:
- [ ] Multi-node selection (show messages from multiple nodes)
- [ ] Node name search/autocomplete for many nodes
- [ ] Save filter presets
- [ ] Node grouping by namespace
- [ ] Color-code nodes
- [ ] Show node description/metadata
- [ ] Export filtered logs
- [ ] Node activity indicators (active/inactive)

## Debugging

Access from console:
```javascript
// View all discovered nodes
GrizzlyDebug.terminal.nodes

// Check active filter
GrizzlyDebug.terminal.activeNodeFilter

// Manually set filter
GrizzlyDebug.terminal.setNodeFilter('my_node')

// Get messages from specific node
GrizzlyDebug.terminal.messages.filter(m => m.name === 'my_node')
```

## Keyboard Shortcuts (Future)

Could add:
- `Ctrl+F`: Focus node filter
- `Ctrl+Shift+C`: Clear all filters
- `Escape`: Clear node filter

## Accessibility

- Dropdown is keyboard navigable
- Clear labels for screen readers
- High contrast maintained
- Focus styles on interactive elements
