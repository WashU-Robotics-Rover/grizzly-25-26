/**
 * Utility Helper Functions
 * Common utilities used across the application
 */

const Utils = {
    /**
     * Format ROS timestamp to readable string
     * @param {Object} timestamp - ROS timestamp with sec and nanosec
     * @returns {string} Formatted date string
     */
    formatROSTime(timestamp) {
        if (!timestamp) return 'N/A';
        const date = new Date(
            timestamp.sec * 1000 + 
            timestamp.nanosec / 1000000
        );
        return date.toLocaleString();
    },

    /**
     * Format current time for display
     * @returns {string} Formatted time string
     */
    formatCurrentTime() {
        return new Date().toLocaleTimeString();
    },

    /**
     * Get DOM element by ID with error handling
     * @param {string} id - Element ID
     * @returns {HTMLElement|null} Element or null
     */
    getElement(id) {
        const element = document.getElementById(id);
        if (!element) {
            console.warn(`Element with id "${id}" not found`);
        }
        return element;
    },

    /**
     * Safely set element text content
     * @param {string} id - Element ID
     * @param {string} text - Text to set
     */
    setText(id, text) {
        const element = this.getElement(id);
        if (element) {
            element.textContent = text;
        }
    },

    /**
     * Safely set element class name
     * @param {string} id - Element ID
     * @param {string} className - Class name to set
     */
    setClass(id, className) {
        const element = this.getElement(id);
        if (element) {
            element.className = className;
        }
    },

    /**
     * Add class to element
     * @param {string} id - Element ID
     * @param {string} className - Class to add
     */
    addClass(id, className) {
        const element = this.getElement(id);
        if (element) {
            element.classList.add(className);
        }
    },

    /**
     * Remove class from element
     * @param {string} id - Element ID
     * @param {string} className - Class to remove
     */
    removeClass(id, className) {
        const element = this.getElement(id);
        if (element) {
            element.classList.remove(className);
        }
    },

    /**
     * Check if element has class
     * @param {string} id - Element ID
     * @param {string} className - Class to check
     * @returns {boolean} True if element has class
     */
    hasClass(id, className) {
        const element = this.getElement(id);
        return element ? element.classList.contains(className) : false;
    },

    /**
     * Set element style property
     * @param {string} id - Element ID
     * @param {string} property - CSS property
     * @param {string} value - CSS value
     */
    setStyle(id, property, value) {
        const element = this.getElement(id);
        if (element) {
            element.style[property] = value;
        }
    },

    /**
     * Log with timestamp
     * @param {string} level - Log level (info, warn, error)
     * @param {string} message - Message to log
     * @param {*} data - Optional data to log
     */
    log(level, message, data = null) {
        const timestamp = new Date().toISOString();
        const prefix = `[${timestamp}] [${level.toUpperCase()}]`;
        
        if (data !== null) {
            console[level](prefix, message, data);
        } else {
            console[level](prefix, message);
        }
    },

    /**
     * Debounce function calls
     * @param {Function} func - Function to debounce
     * @param {number} wait - Wait time in ms
     * @returns {Function} Debounced function
     */
    debounce(func, wait) {
        let timeout;
        return function executedFunction(...args) {
            const later = () => {
                clearTimeout(timeout);
                func(...args);
            };
            clearTimeout(timeout);
            timeout = setTimeout(later, wait);
        };
    },

    /**
     * Throttle function calls
     * @param {Function} func - Function to throttle
     * @param {number} limit - Time limit in ms
     * @returns {Function} Throttled function
     */
    throttle(func, limit) {
        let inThrottle;
        return function(...args) {
            if (!inThrottle) {
                func.apply(this, args);
                inThrottle = true;
                setTimeout(() => inThrottle = false, limit);
            }
        };
    }
};
