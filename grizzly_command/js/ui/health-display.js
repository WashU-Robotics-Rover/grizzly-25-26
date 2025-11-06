/**
 * Health Display UI Manager
 * Handles visualization of system health
 */

const HealthDisplay = {
    /**
     * Update health display with new health data
     * @param {Object} healthMsg - Health message from ROS
     */
    update(healthMsg) {
        const healthData = healthMsg.data || 'No health data';
        
        // Update message and restore normal styling
        this._updateMessage(healthData);
        
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
        
        const healthCircle = Utils.getElement('healthCircle');
        if (healthCircle) {
            healthCircle.style.stroke = Config.health.healthyColor;
        }
    }
};
