/**
 * Configuration and Constants
 * Central location for all application configuration
 */

const Config = {
    // ROS Bridge Configuration
    rosbridge: {
        defaultUrl: 'ws://localhost:9090',
        reconnectDelay: 500
    },

    // State Management
    states: {
        STARTUP: 0,
        STANDBY: 1,
        AUTONOMOUS: 2,
        MANUAL: 3,
        EMERGENCY: 4,
        ERROR: 5,
        SHUTDOWN: 6,
        UNKNOWN: 99
    },

    // State name mapping (reverse lookup)
    stateNames: {
        0: 'STARTUP',
        1: 'STANDBY',
        2: 'AUTONOMOUS',
        3: 'MANUAL',
        4: 'EMERGENCY',
        5: 'ERROR',
        6: 'SHUTDOWN',
        99: 'UNKNOWN'
    },

    // State color schemes (Tailwind classes)
    stateColors: {
        0: { bg: 'bg-zinc-800', border: 'border-zinc-700', text: 'text-zinc-300' },      // STARTUP
        1: { bg: 'bg-blue-900', border: 'border-blue-700', text: 'text-blue-300' },      // STANDBY
        2: { bg: 'bg-green-900', border: 'border-green-700', text: 'text-green-300' },   // AUTONOMOUS
        3: { bg: 'bg-yellow-900', border: 'border-yellow-700', text: 'text-yellow-300' }, // MANUAL
        4: { bg: 'bg-red-900', border: 'border-red-700', text: 'text-red-300' },         // EMERGENCY
        5: { bg: 'bg-red-950', border: 'border-red-800', text: 'text-red-400' },         // ERROR
        6: { bg: 'bg-black', border: 'border-zinc-800', text: 'text-zinc-500' },         // SHUTDOWN
        99: { bg: 'bg-zinc-900', border: 'border-zinc-800', text: 'text-zinc-300' }      // UNKNOWN
    },

    // Health Monitoring
    health: {
        timeoutMs: 5000,  // 5 seconds without health update triggers warning
        warningColor: '#f39c12',
        errorColor: '#e74c3c',
        healthyColor: '#4CAF50'
    },

    // History Management
    history: {
        maxItems: 20
    },

    // ROS Topics
    topics: {
        state: {
            name: '/system/state',
            messageType: 'grizzly_interfaces/msg/OperationalState'
        },
        health: {
            name: '/system/health',
            messageType: 'std_msgs/msg/String'
        },
        rosout: {
            name: '/rosout',
            messageType: 'rcl_interfaces/msg/Log'
        }
    },

    // ROS Services
    services: {
        changeState: {
            name: '/system/change_state',
            serviceType: 'grizzly_interfaces/srv/ChangeState'
        }
    },

    // UI Configuration
    ui: {
        notificationDuration: 5000,  // Auto-hide notifications after 5 seconds
        defaultStateDescription: 'No description',
        connectionCheckInterval: 1000
    }
};

// Freeze config to prevent accidental modifications
Object.freeze(Config);
Object.freeze(Config.rosbridge);
Object.freeze(Config.states);
Object.freeze(Config.stateNames);
Object.freeze(Config.stateColors);
Object.freeze(Config.health);
Object.freeze(Config.history);
Object.freeze(Config.topics);
Object.freeze(Config.services);
Object.freeze(Config.ui);
