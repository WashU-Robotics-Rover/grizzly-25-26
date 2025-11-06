/**
 * Notification System
 * Handles user notifications and service response messages
 */

const Notifications = {
    /**
     * Show service response message in the UI
     * @param {string} message - Message to display
     * @param {boolean} success - Whether the operation was successful
     */
    showServiceResponse(message, success) {
        const responseDiv = Utils.getElement('serviceResponse');
        if (!responseDiv) return;

        responseDiv.textContent = message;
        responseDiv.className = success ? 'success' : 'error';
        responseDiv.classList.remove('hidden');
        
        // Auto-hide after configured duration
        setTimeout(() => {
            responseDiv.classList.add('hidden');
        }, Config.ui.notificationDuration);
    },

    /**
     * Hide service response message immediately
     */
    hideServiceResponse() {
        const responseDiv = Utils.getElement('serviceResponse');
        if (responseDiv && !responseDiv.classList.contains('hidden')) {
            responseDiv.classList.add('hidden');
        }
    },

    /**
     * Show general notification (console-based, can be expanded)
     * @param {string} message - Message to display
     * @param {string} type - Type of notification (info, warning, error, success)
     */
    show(message, type = 'info') {
        Utils.log(type, message);
        // TODO: Could add toast notification system here
    },

    /**
     * Show success notification
     * @param {string} message - Success message
     */
    success(message) {
        this.show(message, 'info');
        this.showServiceResponse(message, true);
    },

    /**
     * Show error notification
     * @param {string} message - Error message
     */
    error(message) {
        this.show(message, 'error');
        this.showServiceResponse(message, false);
    },

    /**
     * Show warning notification
     * @param {string} message - Warning message
     */
    warning(message) {
        this.show(message, 'warn');
        this.showServiceResponse(`⚠️ ${message}`, false);
    },

    /**
     * Show info notification
     * @param {string} message - Info message
     */
    info(message) {
        this.show(message, 'info');
    }
};
