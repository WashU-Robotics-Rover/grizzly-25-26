/**
 * ROS Connection Manager
 * Handles connection lifecycle to ROS Bridge
 */

const ROSConnection = {
    ros: null,
    isConnected: false,
    callbacks: {
        onConnect: [],
        onDisconnect: [],
        onError: []
    },

    /**
     * Initialize ROS connection
     * @param {string} url - WebSocket URL for rosbridge
     */
    connect(url = null) {
        const rosBridgeUrl = url || Utils.getElement('rosBridgeUrl')?.value || Config.rosbridge.defaultUrl;
        
        Utils.log('info', 'Connecting to ROS Bridge', rosBridgeUrl);
        
        // Create ROS connection
        this.ros = new ROSLIB.Ros({
            url: rosBridgeUrl
        });
        
        // Connection event handlers
        this.ros.on('connection', () => {
            Utils.log('info', 'Connected to ROS Bridge!');
            this.isConnected = true;
            this._updateConnectionUI(true);
            this._triggerCallbacks('onConnect');
        });
        
        this.ros.on('error', (error) => {
            Utils.log('error', 'ROS Bridge connection error', error);
            this.isConnected = false;
            this._updateConnectionUI(false);
            Notifications.error('Connection Error: ' + error);
            this._triggerCallbacks('onError', error);
        });
        
        this.ros.on('close', () => {
            Utils.log('info', 'Connection to ROS Bridge closed');
            this.isConnected = false;
            this._updateConnectionUI(false);
            Notifications.error('Disconnected from ROS Bridge');
            this._triggerCallbacks('onDisconnect');
        });
    },

    /**
     * Reconnect to ROS Bridge
     */
    reconnect() {
        Utils.log('info', 'Reconnecting to ROS Bridge...');
        if (this.ros) {
            this.ros.close();
        }
        setTimeout(() => this.connect(), Config.rosbridge.reconnectDelay);
    },

    /**
     * Get current ROS instance
     * @returns {ROSLIB.Ros|null} ROS instance
     */
    getRos() {
        return this.ros;
    },

    /**
     * Check if connected to ROS
     * @returns {boolean} Connection status
     */
    getConnectionStatus() {
        return this.isConnected && this.ros && this.ros.isConnected;
    },

    /**
     * Register callback for connection events
     * @param {string} event - Event name (onConnect, onDisconnect, onError)
     * @param {Function} callback - Callback function
     */
    on(event, callback) {
        if (this.callbacks[event]) {
            this.callbacks[event].push(callback);
        }
    },

    /**
     * Trigger registered callbacks
     * @private
     */
    _triggerCallbacks(event, data = null) {
        if (this.callbacks[event]) {
            this.callbacks[event].forEach(callback => {
                try {
                    callback(data);
                } catch (error) {
                    Utils.log('error', `Error in ${event} callback`, error);
                }
            });
        }
    },

    /**
     * Update connection status UI
     * @private
     */
    _updateConnectionUI(connected) {
        const indicator = Utils.getElement('connectionIndicator');
        const ping = Utils.getElement('connectionPing');
        const serviceReady = Utils.getElement('serviceReady');
        const status = Utils.getElement('connectionStatus');
        
        if (connected) {
            if (indicator) {
                indicator.classList.remove('bg-red-500');
                indicator.classList.add('bg-green-500', 'connected');
            }
            if (ping) {
                ping.style.display = 'none';
            }
            if (status) {
                status.textContent = 'Connected';
                status.classList.remove('text-red-400');
                status.classList.add('text-green-400', 'connected');
            }
        } else {
            if (indicator) {
                indicator.classList.remove('bg-green-500', 'connected');
                indicator.classList.add('bg-red-500');
            }
            if (ping) {
                ping.style.display = 'inline-flex';
            }
            if (status) {
                status.textContent = 'Disconnected';
                status.classList.remove('text-green-400', 'connected');
                status.classList.add('text-red-400');
            }
            
            // Reset service status when disconnected
            if (serviceReady) {
                serviceReady.textContent = 'NOT READY';
                serviceReady.className = 'text-red-500';
            }
        }
    }
};
