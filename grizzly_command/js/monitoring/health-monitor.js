/**
 * Health Monitoring System
 * Monitors health updates and detects timeouts
 */

const HealthMonitor = {
    timeoutHandle: null,
    lastHealthTime: null,
    isHealthy: false,

    /**
     * Start health monitoring
     */
    start() {
        this.isHealthy = true;
        this.lastHealthTime = null;
        this.reset();
        Utils.log('info', `Health monitoring started (${Config.health.timeoutMs}ms timeout)`);
    },

    /**
     * Reset the health timeout timer
     */
    reset() {
        // Clear existing timeout
        if (this.timeoutHandle) {
            clearTimeout(this.timeoutHandle);
        }
        
        // Set new timeout
        this.timeoutHandle = setTimeout(
            () => this._onTimeout(),
            Config.health.timeoutMs
        );
    },

    /**
     * Record health update received
     */
    recordUpdate() {
        this.lastHealthTime = new Date();
        this.isHealthy = true;
        this.reset();
    },

    /**
     * Stop health monitoring
     */
    stop() {
        if (this.timeoutHandle) {
            clearTimeout(this.timeoutHandle);
            this.timeoutHandle = null;
        }
        this.isHealthy = false;
        this.lastHealthTime = null;
        Utils.log('info', 'Health monitoring stopped');
    },

    /**
     * Get current health status
     * @returns {boolean} True if healthy
     */
    getHealthStatus() {
        return this.isHealthy;
    },

    /**
     * Get last health update time
     * @returns {Date|null} Last health update time
     */
    getLastUpdateTime() {
        return this.lastHealthTime;
    },

    /**
     * Called when health timeout occurs
     * @private
     */
    _onTimeout() {
        Utils.log('warn', `Health timeout! No updates for ${Config.health.timeoutMs}ms`);
        this.isHealthy = false;
        
        // Update UI to show UNKNOWN state
        StateDisplay.setUnknownState();
        HealthDisplay.setWarningState();
        
        // Show warning notification
        Notifications.warning('WARNING: System health updates lost. Connection may be unstable.');
    }
};
