/**
 * Terminal Output Display
 * Displays ROS rosout messages in a terminal-style interface
 */

const TerminalOutput = {
    messages: [],
    maxMessages: 1000,
    autoscroll: true,
    filters: {
        DEBUG: true,
        INFO: true,
        WARN: true,
        ERROR: true,
        FATAL: true
    },
    stats: {
        DEBUG: 0,
        INFO: 0,
        WARN: 0,
        ERROR: 0,
        FATAL: 0
    },
    nodes: new Set(), // Track all unique node names
    activeNodeFilter: null, // Current node filter (null = show all)

    /**
     * Initialize terminal display
     */
    init() {
        Utils.log('info', 'Terminal output initialized');
        this._updateStats();
        
        // Add test message to verify terminal is working
        this.addTestMessage();
    },

    /**
     * Add a test message to verify terminal is working
     */
    addTestMessage() {
        const testMessage = {
            level: 2, // INFO
            name: 'terminal_test',
            msg: 'Terminal initialized successfully. Waiting for ROS messages...',
            header: {
                stamp: {
                    sec: Math.floor(Date.now() / 1000),
                    nanosec: (Date.now() % 1000) * 1000000
                }
            }
        };
        this.addMessage(testMessage);
    },

    /**
     * Add a rosout message to the terminal
     * @param {Object} message - ROS rosout message
     */
    addMessage(message) {
        // Debug log for first few messages
        if (this.messages.length < 3) {
            console.log('TerminalOutput received message:', message);
        }

        // Store message
        this.messages.push(message);
        
        // Track node name
        const nodeName = message.name || 'unknown';
        if (!this.nodes.has(nodeName)) {
            this.nodes.add(nodeName);
            this._updateNodeFilter();
        }

        // Limit message history
        if (this.messages.length > this.maxMessages) {
            this.messages.shift();
        }

        // Update stats
        const level = this._getLevelName(message.level);
        if (this.stats.hasOwnProperty(level)) {
            this.stats[level]++;
        }

        // Render if filters allow
        if (this._shouldRenderMessage(message)) {
            this._renderMessage(message);
        }

        this._updateStats();

        // Auto-scroll if enabled
        if (this.autoscroll) {
            this.scrollToBottom();
        }
    },
    
    /**
     * Check if message should be rendered based on filters
     * @private
     */
    _shouldRenderMessage(message) {
        const level = this._getLevelName(message.level);
        const nodeName = message.name || 'unknown';
        
        // Check level filter
        if (!this.filters[level]) {
            return false;
        }
        
        // Check node filter
        if (this.activeNodeFilter && nodeName !== this.activeNodeFilter) {
            return false;
        }
        
        return true;
    },

    /**
     * Render a single message to the terminal
     * @private
     */
    _renderMessage(message) {
        const terminalOutput = Utils.getElement('terminalOutput');
        if (!terminalOutput) {
            console.error('Terminal output element not found');
            return;
        }

        // Remove placeholder if present (only remove div placeholders, not timestamp spans)
        const placeholder = terminalOutput.querySelector('div.text-zinc-600');
        if (placeholder) {
            placeholder.remove();
        }

        try {
            const level = this._getLevelName(message.level);
            const colors = this._getLevelColors(level);
            const timestamp = this._formatTimestamp(message.header ? message.header.stamp : message.stamp);
            const nodeName = message.name || 'unknown';
            const msgText = message.msg || message.message || String(message);

            // Create message element
            const messageDiv = document.createElement('div');
            messageDiv.className = `terminal-line ${colors.text} text-xs leading-tight mb-2`;

            messageDiv.dataset.level = level;
            messageDiv.dataset.node = nodeName;

            // Format: timestamp on left, then [LEVEL] [node] message
            messageDiv.innerHTML = `<span class="text-zinc-600 inline-block w-24">${timestamp}</span><span class="${colors.badge}">[${level}]</span> <span class="text-zinc-500">[${nodeName}]</span> <span class="${colors.text}">${this._escapeHtml(msgText)}</span>`;

            terminalOutput.appendChild(messageDiv);
        } catch (error) {
            console.error('Error rendering terminal message:', error, message);
        }
    },

    /**
     * Clear all terminal messages
     */
    clear() {
        this.messages = [];
        this.nodes.clear();
        this.activeNodeFilter = null;
        this.stats = {
            DEBUG: 0,
            INFO: 0,
            WARN: 0,
            ERROR: 0,
            FATAL: 0
        };

        const terminalOutput = Utils.getElement('terminalOutput');
        if (terminalOutput) {
            terminalOutput.innerHTML = '<div class="text-zinc-600">// Terminal cleared</div>';
        }

        // Reset node filter dropdown
        const filterSelect = Utils.getElement('nodeFilterSelect');
        if (filterSelect) {
            filterSelect.innerHTML = '<option value="">ALL NODES</option>';
            filterSelect.value = '';
        }

        this._updateStats();
        Utils.log('info', 'Terminal cleared');
    },

    /**
     * Toggle autoscroll
     */
    toggleAutoscroll() {
        this.autoscroll = !this.autoscroll;
        
        const indicator = Utils.getElement('autoscrollIndicator');
        if (indicator) {
            indicator.textContent = `[AUTO-SCROLL: ${this.autoscroll ? 'ON' : 'OFF'}]`;
        }

        Utils.log('info', `Auto-scroll ${this.autoscroll ? 'enabled' : 'disabled'}`);
    },

    /**
     * Toggle log level filter
     * @param {string} level - Log level to toggle
     */
    toggleLevel(level) {
        if (!this.filters.hasOwnProperty(level)) return;

        this.filters[level] = !this.filters[level];

        // Update button style
        const button = Utils.getElement(`filter${level.charAt(0) + level.slice(1).toLowerCase()}`);
        if (button) {
            if (this.filters[level]) {
                button.classList.add('active');
            } else {
                button.classList.remove('active');
            }
        }

        // Re-render all messages
        this._rerenderAll();

        Utils.log('info', `${level} filter ${this.filters[level] ? 'enabled' : 'disabled'}`);
    },

    /**
     * Scroll terminal to bottom
     */
    scrollToBottom() {
        const container = Utils.getElement('terminalContainer');
        if (container) {
            container.scrollTop = container.scrollHeight;
        }
    },

    /**
     * Set node filter
     * @param {string|null} nodeName - Node name to filter by, or null for all
     */
    setNodeFilter(nodeName) {
        this.activeNodeFilter = nodeName;
        this._rerenderAll();
        
        // Update UI
        const filterSelect = Utils.getElement('nodeFilterSelect');
        if (filterSelect) {
            filterSelect.value = nodeName || '';
        }
        
        Utils.log('info', `Node filter set to: ${nodeName || 'ALL'}`);
    },
    
    /**
     * Update node filter dropdown
     * @private
     */
    _updateNodeFilter() {
        const filterSelect = Utils.getElement('nodeFilterSelect');
        if (!filterSelect) return;
        
        const currentValue = filterSelect.value;
        
        // Rebuild options
        filterSelect.innerHTML = '<option value="">ALL NODES</option>';
        
        Array.from(this.nodes).sort().forEach(nodeName => {
            const option = document.createElement('option');
            option.value = nodeName;
            option.textContent = nodeName;
            filterSelect.appendChild(option);
        });
        
        // Restore selection
        filterSelect.value = currentValue;
    },

    /**
     * Re-render all messages with current filters
     * @private
     */
    _rerenderAll() {
        const terminalOutput = Utils.getElement('terminalOutput');
        if (!terminalOutput) return;

        terminalOutput.innerHTML = '';

        if (this.messages.length === 0) {
            terminalOutput.innerHTML = '<div class="text-zinc-600">// No messages</div>';
            return;
        }

        // Render filtered messages
        this.messages.forEach(message => {
            if (this._shouldRenderMessage(message)) {
                this._renderMessage(message);
            }
        });

        if (terminalOutput.children.length === 0) {
            terminalOutput.innerHTML = '<div class="text-zinc-600">// All messages filtered out</div>';
        }
    },

    /**
     * Update statistics display
     * @private
     */
    _updateStats() {
        Utils.setText('terminalMessageCount', this.messages.length);
        Utils.setText('terminalNodeCount', this.nodes.size);
        Utils.setText('terminalDebugCount', this.stats.DEBUG);
        Utils.setText('terminalInfoCount', this.stats.INFO);
        Utils.setText('terminalWarnCount', this.stats.WARN);
        Utils.setText('terminalErrorCount', this.stats.ERROR);
        Utils.setText('terminalFatalCount', this.stats.FATAL);
    },

    /**
     * Get level name from level number
     * @private
     */
    _getLevelName(level) {
        const levels = {
            1: 'DEBUG',
            2: 'INFO',
            4: 'WARN',
            8: 'ERROR',
            16: 'FATAL'
        };
        return levels[level] || 'INFO';
    },

    /**
     * Get colors for log level
     * @private
     */
    _getLevelColors(level) {
        const colors = {
            DEBUG: { text: 'text-blue-400', badge: 'text-blue-500' },
            INFO: { text: 'text-green-400', badge: 'text-green-500' },
            WARN: { text: 'text-yellow-400', badge: 'text-yellow-500' },
            ERROR: { text: 'text-red-400', badge: 'text-red-500' },
            FATAL: { text: 'text-red-500', badge: 'text-red-600' }
        };
        return colors[level] || colors.INFO;
    },

    /**
     * Format ROS timestamp for display
     * @private
     */
    _formatTimestamp(stamp) {
        if (!stamp) {
            // Use current time if no stamp provided
            const now = new Date();
            const hours = String(now.getHours()).padStart(2, '0');
            const minutes = String(now.getMinutes()).padStart(2, '0');
            const seconds = String(now.getSeconds()).padStart(2, '0');
            const millis = String(now.getMilliseconds()).padStart(3, '0');
            return `${hours}:${minutes}:${seconds}.${millis}`;
        }
        
        try {
            let date;
            if (stamp.sec !== undefined && stamp.nanosec !== undefined) {
                // ROS2 format: {sec, nanosec}
                date = new Date(stamp.sec * 1000 + stamp.nanosec / 1000000);
            } else if (stamp.secs !== undefined && stamp.nsecs !== undefined) {
                // ROS1 format: {secs, nsecs}
                date = new Date(stamp.secs * 1000 + stamp.nsecs / 1000000);
            } else {
                // Unknown format, use current time
                date = new Date();
            }
            
            const hours = String(date.getHours()).padStart(2, '0');
            const minutes = String(date.getMinutes()).padStart(2, '0');
            const seconds = String(date.getSeconds()).padStart(2, '0');
            const millis = String(date.getMilliseconds()).padStart(3, '0');
            
            return `${hours}:${minutes}:${seconds}.${millis}`;
        } catch (error) {
            console.error('Error formatting timestamp:', error, stamp);
            return '--:--:--.---';
        }
    },

    /**
     * Escape HTML in message text
     * @private
     */
    _escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    },

    /**
     * Export terminal messages as JSON
     * @returns {string} JSON string of messages
     */
    exportJSON() {
        return JSON.stringify(this.messages, null, 2);
    },

    /**
     * Export terminal messages as text
     * @returns {string} Plain text log
     */
    exportText() {
        return this.messages.map(msg => {
            const level = this._getLevelName(msg.level);
            const timestamp = this._formatTimestamp(msg.header ? msg.header.stamp : msg.stamp);
            const nodeName = msg.name || 'unknown';
            return `[${timestamp}] [${level}] [${nodeName}] ${msg.msg}`;
        }).join('\n');
    },

    /**
     * Add a manual test message (for debugging)
     * @param {string} text - Message text
     * @param {number} level - Log level (1=DEBUG, 2=INFO, 4=WARN, 8=ERROR, 16=FATAL)
     */
    addManualMessage(text, level = 2) {
        const message = {
            level: level,
            name: 'manual_test',
            msg: text,
            header: {
                stamp: {
                    sec: Math.floor(Date.now() / 1000),
                    nanosec: (Date.now() % 1000) * 1000000
                }
            }
        };
        this.addMessage(message);
        console.log('Manual message added:', message);
    }
};

