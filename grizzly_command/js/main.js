/**
 * Main Application Entry Point
 * Orchestrates all modules and handles initialization
 */

const GrizzlyApp = {
    /**
     * Initialize the application
     */
    init() {
        Utils.log('info', 'Initializing Grizzly Rover Web Interface...');
        
        // Initialize UI modules
        TerminalOutput.init();
        
        // Setup connection event handlers
        this._setupConnectionHandlers();
        
        // Setup subscriber callbacks
        this._setupSubscriberCallbacks();
        
        // Connect to ROS
        ROSConnection.connect();
        
        // Expose global functions for HTML onclick handlers
        this._exposeGlobalFunctions();
        
        Utils.log('info', 'Application initialized successfully');
    },

    /**
     * Setup ROS connection event handlers
     * @private
     */
    _setupConnectionHandlers() {
        ROSConnection.on('onConnect', () => {
            Utils.log('info', 'Connection established - setting up ROS interface');
            ROSSubscribers.setup();
            ROSServices.setup();
            HealthMonitor.start();
        });

        ROSConnection.on('onDisconnect', () => {
            Utils.log('info', 'Connection lost - cleaning up');
            ROSSubscribers.cleanup();
            ROSServices.cleanup();
            HealthMonitor.stop();
        });

        ROSConnection.on('onError', (error) => {
            Utils.log('error', 'Connection error occurred', error);
        });
    },

    /**
     * Setup subscriber callbacks
     * @private
     */
    _setupSubscriberCallbacks() {
        // State update handler
        ROSSubscribers.on('onStateUpdate', (message) => {
            StateDisplay.update(message);
            HistoryManager.add(message);
        });

        // Health update handler
        ROSSubscribers.on('onHealthUpdate', (message) => {
            HealthDisplay.update(message);
            HealthMonitor.recordUpdate();
        });

        // Rosout update handler
        ROSSubscribers.on('onRosoutUpdate', (message) => {
            console.log('Main.js: Rosout callback triggered');
            TerminalOutput.addMessage(message);
        });
        
        Utils.log('info', 'Rosout callback registered');
    },

    /**
     * Expose functions to global scope for HTML onclick handlers
     * @private
     */
    _exposeGlobalFunctions() {
        window.requestStateChange = (targetState, reason) => {
            ROSServices.requestStateChange(targetState, reason)
                .then(response => {
                    Utils.log('info', 'State change completed successfully', response);
                })
                .catch(error => {
                    Utils.log('error', 'State change failed', error);
                });
        };

        window.reconnect = () => {
            ROSConnection.reconnect();
        };

        // Expose TerminalOutput for HTML onclick handlers
        window.TerminalOutput = TerminalOutput;

        // Expose modules for debugging/advanced features
        window.GrizzlyDebug = {
            config: Config,
            connection: ROSConnection,
            subscribers: ROSSubscribers,
            services: ROSServices,
            stateDisplay: StateDisplay,
            healthDisplay: HealthDisplay,
            history: HistoryManager,
            healthMonitor: HealthMonitor,
            terminal: TerminalOutput,
            tabManager: TabManager,
            utils: Utils,
            notifications: Notifications
        };

        Utils.log('info', 'Global functions exposed (window.requestStateChange, window.reconnect, window.TerminalOutput, window.GrizzlyDebug)');
    },

    /**
     * Reconnect to ROS (convenience method)
     */
    reconnect() {
        ROSConnection.reconnect();
    },

    /**
     * Get application status
     * @returns {Object} Application status information
     */
    getStatus() {
        return {
            connected: ROSConnection.getConnectionStatus(),
            healthy: HealthMonitor.getHealthStatus(),
            lastHealthUpdate: HealthMonitor.getLastUpdateTime(),
            historySize: HistoryManager.getHistory().length
        };
    }
};

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    GrizzlyApp.init();
});
