/**
 * Health Display UI Manager
 * Handles visualization of system health
 */

const HealthDisplay = {
    // Track last update time for latency calculation
    _lastUpdateTime: null,
    _expectedInterval: 1000, // Expected interval between health messages in ms
    
    /**
     * Update health display with new health data
     * @param {Object} healthMsg - Health message from ROS
     */
    update(healthMsg) {
        const healthData = healthMsg.data || 'No health data';
        
        // Update message and restore normal styling
        this._updateMessage(healthData);
        
        // Calculate and update latency
        this._updateLatency();
        
        // Update timestamp
        this._updateTimestamp();
        
        // Update health indicator based on message content
        this._updateIndicator(healthData);
        
        // Clear any connection warnings
        Notifications.hideServiceResponse();
        
        Utils.log('info', `Health updated: ${healthData}`);
    },

    /**
     * Set health to warning state (no updates received)
     */
    setWarningState() {
        const healthMessage = Utils.getElement('healthMessage');
        if (healthMessage) {
            healthMessage.textContent = '⚠️ NO HEALTH UPDATES RECEIVED';
            healthMessage.className = 'text-xs text-yellow-300 font-mono font-bold';
        }
        
        const healthCircle = Utils.getElement('healthCircle');
        if (healthCircle) {
            healthCircle.style.stroke = Config.health.warningColor;
        }
    },

    /**
     * Update health message
     * @private
     */
    _updateMessage(healthData) {
        const healthMessage = Utils.getElement('healthMessage');
        if (healthMessage) {
            healthMessage.textContent = healthData;
            healthMessage.className = 'text-xs text-zinc-300 font-mono';
        }
    },

    /**
     * Calculate and update latency display
     * @private
     */
    _updateLatency() {
        const currentTime = Date.now();
        const healthLatency = Utils.getElement('healthLatency');
        
        if (!healthLatency) return;
        
        if (this._lastUpdateTime === null) {
            // First update - no latency to calculate yet
            healthLatency.textContent = '--';
            healthLatency.className = 'text-zinc-400';
            this._lastUpdateTime = currentTime;
            return;
        }
        
        // Calculate latency (time since last update)
        const latency = currentTime - this._lastUpdateTime - 1000;
        this._lastUpdateTime = currentTime;
        
        // Update latency display
        if (latency < 0) {
            latency = 0;
        }

        healthLatency.textContent = Math.round(latency);
        
        // Color-code based on deviation from expected 1000ms interval
        const deviation = Math.abs(latency);
        
        if (deviation < 100) {
            // Within 100ms of expected - excellent (green)
            healthLatency.className = 'text-green-400';
        } else if (deviation < 250) {
            // Within 250ms of expected - good (blue)
            healthLatency.className = 'text-blue-400';
        } else if (deviation < 500) {
            // Within 500ms of expected - warning (yellow)
            healthLatency.className = 'text-yellow-400';
        } else {
            // More than 500ms deviation - poor (red)
            healthLatency.className = 'text-red-400';
        }
    },
    
    /**
     * Update health timestamp
     * @private
     */
    _updateTimestamp() {
        const healthTimestamp = Utils.getElement('healthTimestamp');
        if (healthTimestamp) {
            healthTimestamp.textContent = Utils.formatCurrentTime();
        }
    },

    /**
     * Update health indicator based on message content
     * @private
     */
    _updateIndicator(healthData) {
        const healthCircle = Utils.getElement('healthCircle');
        if (!healthCircle) return;

        const data = healthData.toLowerCase();
        
        if (data.includes('error') || data.includes('critical')) {
            healthCircle.style.stroke = Config.health.errorColor;
        } else if (data.includes('warning') || data.includes('degraded')) {
            healthCircle.style.stroke = Config.health.warningColor;
        } else {
            healthCircle.style.stroke = Config.health.healthyColor;
        }
    },

    /**
     * Reset health display to waiting state
     */
    reset() {
        Utils.setText('healthMessage', 'WAITING FOR DATA...');
        Utils.setClass('healthMessage', 'text-xs text-zinc-300 font-mono');
        
        // Reset latency tracking
        this._lastUpdateTime = null;
        Utils.setText('healthLatency', '--');
        Utils.setClass('healthLatency', 'text-zinc-400');
        
        const healthCircle = Utils.getElement('healthCircle');
        if (healthCircle) {
            healthCircle.style.stroke = Config.health.healthyColor;
        }
    }
};
