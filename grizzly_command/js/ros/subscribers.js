/**
 * ROS Topic Subscribers
 * Manages subscriptions to ROS topics
 */

const ROSSubscribers = {
    stateListener: null,
    healthListener: null,
    rosoutListener: null,
    callbacks: {
        onStateUpdate: [],
        onHealthUpdate: [],
        onRosoutUpdate: []
    },

    /**
     * Setup all ROS topic subscriptions
     */
    setup() {
        this._cleanupExistingSubscribers();
        this._resetUI();
        this._subscribeToState();
        this._subscribeToHealth();
        this._subscribeToRosout();
    },

    /**
     * Cleanup existing subscribers
     * @private
     */
    _cleanupExistingSubscribers() {
        if (this.stateListener) {
            Utils.log('info', 'Unsubscribing from old state listener');
            this.stateListener.unsubscribe();
            this.stateListener = null;
        }
        
        if (this.healthListener) {
            Utils.log('info', 'Unsubscribing from old health listener');
            this.healthListener.unsubscribe();
            this.healthListener = null;
        }
        
        if (this.rosoutListener) {
            Utils.log('info', 'Unsubscribing from old rosout listener');
            this.rosoutListener.unsubscribe();
            this.rosoutListener = null;
        }
    },

    /**
     * Reset UI to initial state
     * @private
     */
    _resetUI() {
        // Clear any connection warning messages
        Notifications.hideServiceResponse();
        
        // Reset health display
        Utils.setText('healthMessage', 'WAITING FOR DATA...');
        Utils.setClass('healthMessage', 'text-xs text-zinc-300 font-mono');
        
        // Reset state badge
        Utils.setClass('stateBadge', 'bg-zinc-900 border border-zinc-800 p-6 mb-3');
        Utils.setText('stateName', 'CONNECTING...');
        Utils.setClass('stateName', 'text-3xl font-bold text-zinc-400 mb-1 tracking-wide');
        Utils.setText('stateDescription', 'Waiting for state update...');
        Utils.setClass('stateDescription', 'text-zinc-300');
    },

    /**
     * Subscribe to state topic
     * @private
     */
    _subscribeToState() {
        const ros = ROSConnection.getRos();
        if (!ros) {
            Utils.log('error', 'Cannot subscribe to state: ROS not connected');
            return;
        }

        this.stateListener = new ROSLIB.Topic({
            ros: ros,
            name: Config.topics.state.name,
            messageType: Config.topics.state.messageType
        });
        
        this.stateListener.subscribe((message) => {
            Utils.log('info', 'Received state update', message);
            this._triggerCallbacks('onStateUpdate', message);
        });

        Utils.log('info', `Subscribed to ${Config.topics.state.name}`);
    },

    /**
     * Subscribe to health topic
     * @private
     */
    _subscribeToHealth() {
        const ros = ROSConnection.getRos();
        if (!ros) {
            Utils.log('error', 'Cannot subscribe to health: ROS not connected');
            return;
        }

        this.healthListener = new ROSLIB.Topic({
            ros: ros,
            name: Config.topics.health.name,
            messageType: Config.topics.health.messageType
        });
        
        this.healthListener.subscribe((message) => {
            Utils.log('info', 'Received health update', message);
            this._triggerCallbacks('onHealthUpdate', message);
        });

        Utils.log('info', `Subscribed to ${Config.topics.health.name}`);
    },

    /**
     * Subscribe to rosout topic
     * @private
     */
    _subscribeToRosout() {
        const ros = ROSConnection.getRos();
        if (!ros) {
            Utils.log('error', 'Cannot subscribe to rosout: ROS not connected');
            return;
        }

        let messageCount = 0;
        this.rosoutListener = new ROSLIB.Topic({
            ros: ros,
            name: Config.topics.rosout.name,
            messageType: Config.topics.rosout.messageType
        });
        
        this.rosoutListener.subscribe((message) => {
            messageCount++;
            // Log first few messages for debugging
            if (messageCount <= 3) {
                Utils.log('info', `Rosout message ${messageCount} received:`, message);
            }
            this._triggerCallbacks('onRosoutUpdate', message);
        });

        Utils.log('info', `Subscribed to ${Config.topics.rosout.name}`);
    },

    /**
     * Register callback for subscription events
     * @param {string} event - Event name (onStateUpdate, onHealthUpdate)
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
    _triggerCallbacks(event, data) {
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
     * Cleanup all subscriptions
     */
    cleanup() {
        this._cleanupExistingSubscribers();
    }
};
