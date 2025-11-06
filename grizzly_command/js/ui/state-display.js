/**
 * State Display UI Manager
 * Handles visualization of operational state
 */

const StateDisplay = {
    /**
     * Update state display with new state data
     * @param {Object} stateMsg - State message from ROS
     */
    update(stateMsg) {
        const stateNameStr = Config.stateNames[stateMsg.state] || 'UNKNOWN';
        const colors = Config.stateColors[stateMsg.state] || Config.stateColors[99];
        
        // Update badge styling
        this._updateBadge(colors);
        
        // Update state name
        this._updateStateName(stateNameStr, colors);
        
        // Update description
        this._updateDescription(stateMsg.description);
        
        // Update metadata
        this._updateMetadata(stateMsg);
        
        // Clear any connection warnings
        Notifications.hideServiceResponse();
        
        Utils.log('info', `State updated to: ${stateNameStr}`);
    },

    /**
     * Set state to UNKNOWN (for health timeouts)
     */
    setUnknownState() {
        const stateBadge = Utils.getElement('stateBadge');
        const stateName = Utils.getElement('stateName');
        const stateDescription = Utils.getElement('stateDescription');
        
        if (stateBadge) {
            stateBadge.className = 'bg-zinc-900 border-2 border-yellow-500 p-6 mb-3 animate-pulse';
        }
        
        if (stateName) {
            stateName.textContent = 'UNKNOWN';
            stateName.className = 'text-3xl font-bold text-yellow-300 mb-1 tracking-wide';
        }
        
        if (stateDescription) {
            stateDescription.textContent = 'System health updates lost';
            stateDescription.className = 'text-zinc-300';
        }
    },

    /**
     * Update badge styling
     * @private
     */
    _updateBadge(colors) {
        const stateBadge = Utils.getElement('stateBadge');
        if (stateBadge) {
            stateBadge.className = `${colors.bg} border ${colors.border} p-6 mb-3`;
        }
    },

    /**
     * Update state name display
     * @private
     */
    _updateStateName(stateNameStr, colors) {
        const stateName = Utils.getElement('stateName');
        if (stateName) {
            stateName.textContent = stateNameStr;
            stateName.className = `text-3xl font-bold ${colors.text} mb-1 tracking-wide`;
        }
    },

    /**
     * Update state description
     * @private
     */
    _updateDescription(description) {
        const stateDescription = Utils.getElement('stateDescription');
        if (stateDescription) {
            stateDescription.textContent = description || Config.ui.defaultStateDescription;
            stateDescription.className = 'text-zinc-300';
        }
    },

    /**
     * Update state metadata (timestamp, value)
     * @private
     */
    _updateMetadata(stateMsg) {
        // Update state value
        Utils.setText('stateValue', stateMsg.state);
        
        // Update timestamp
        const stateTimestamp = Utils.getElement('stateTimestamp');
        if (stateTimestamp) {
            if (stateMsg.timestamp) {
                stateTimestamp.textContent = Utils.formatROSTime(stateMsg.timestamp);
            } else {
                stateTimestamp.textContent = new Date().toLocaleString();
            }
        }
    },

    /**
     * Get state name by state number
     * @param {number} state - State number
     * @returns {string} State name
     */
    getStateName(state) {
        return Config.stateNames[state] || 'UNKNOWN';
    }
};
