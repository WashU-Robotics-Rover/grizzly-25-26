/**
 * ROS Service Clients
 * Manages ROS service calls
 */

const ROSServices = {
    stateChangeClient: null,

    /**
     * Setup all ROS service clients
     */
    setup() {
        this._setupStateChangeService();
    },

    /**
     * Setup state change service client
     * @private
     */
    _setupStateChangeService() {
        const ros = ROSConnection.getRos();
        if (!ros) {
            Utils.log('error', 'Cannot setup service: ROS not connected');
            return;
        }

        try {
            this.stateChangeClient = new ROSLIB.Service({
                ros: ros,
                name: Config.services.changeState.name,
                serviceType: Config.services.changeState.serviceType
            });
            
            Utils.log('info', 'State change service client initialized');
            
            // Update UI to show service is ready
            const serviceReady = Utils.getElement('serviceReady');
            if (serviceReady) {
                serviceReady.textContent = 'READY';
                serviceReady.className = 'text-green-500';
            }
        } catch (error) {
            Utils.log('error', 'Failed to initialize service client', error);
            const serviceReady = Utils.getElement('serviceReady');
            if (serviceReady) {
                serviceReady.textContent = 'ERROR';
                serviceReady.className = 'text-red-500';
            }
        }
    },

    /**
     * Request a state change via ROS service
     * @param {number} targetState - Target state number
     * @param {string} reason - Reason for state change
     * @returns {Promise} Promise that resolves with service response
     */
    requestStateChange(targetState, reason) {
        return new Promise((resolve, reject) => {
            Utils.log('info', 'Requesting state change', { targetState, reason });
            
            // Validate connection
            if (!ROSConnection.getConnectionStatus()) {
                const errorMsg = 'Not connected to ROS Bridge. Please connect first.';
                Notifications.error(errorMsg);
                reject(new Error(errorMsg));
                return;
            }
            
            // Validate service client
            if (!this.stateChangeClient) {
                const errorMsg = 'State change service not initialized. Check console for details.';
                Notifications.error(errorMsg);
                reject(new Error(errorMsg));
                return;
            }
            
            // Create service request
            const request = new ROSLIB.ServiceRequest({
                requested_state: targetState,
                reason: reason
            });
            
            // Call service
            this.stateChangeClient.callService(
                request,
                (response) => {
                    Utils.log('info', 'State change service response', response);
                    
                    if (response.success) {
                        Notifications.success(
                            `✓ State change successful: ${response.message}`
                        );
                        resolve(response);
                    } else {
                        Notifications.error(
                            `✗ State change failed: ${response.message}`
                        );
                        reject(new Error(response.message));
                    }
                },
                (error) => {
                    Utils.log('error', 'State change service call failed', error);
                    Notifications.error(
                        `Service call error: ${error}`
                    );
                    reject(error);
                }
            );
        });
    },

    /**
     * Cleanup service clients
     */
    cleanup() {
        this.stateChangeClient = null;
        
        const serviceReady = Utils.getElement('serviceReady');
        if (serviceReady) {
            serviceReady.textContent = 'NOT READY';
            serviceReady.className = 'text-red-500';
        }
    }
};
