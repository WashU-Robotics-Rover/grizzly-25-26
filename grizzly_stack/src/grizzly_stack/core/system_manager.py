"""
System Manager Node for Grizzly Robotics Stack

This module implements a ROS2 Lifecycle Node that monitors and reports system health.
Lifecycle nodes provide deterministic state management with defined transitions between
states (Unconfigured -> Inactive -> Active -> etc.)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn, State
from grizzly_interfaces.msg import NodeStatus, OperationalState
from grizzly_interfaces.srv import ChangeState


class SystemManager(LifecycleNode):
    """
    SystemManager is a ROS2 Lifecycle Node that manages system health reporting.
    
    Lifecycle States:
    - Unconfigured: Initial state, no resources allocated
    - Inactive: Configured but not running (resources allocated but not active)
    - Active: Fully operational, publishing health status
    - Finalized: Shutdown complete
    
    This node publishes periodic health status messages to '/system/health' topic
    when in the Active state.
    """
    def __init__(self):
        """
        Initialize the SystemManager node in the Unconfigured state.
        
        This constructor is called when the node is first created. At this stage,
        we only declare parameters and initialize member variables to None.
        No active resources (publishers, timers) are created yet.
        """
        # Call the parent LifecycleNode constructor with the node name
        super().__init__('system_manager')
        
        # Declare a ROS2 parameter for the health check rate (in Hz)
        # Default value is 1.0 Hz (one message per second)
        # This can be overridden via launch files or command line
        self.declare_parameter('health_rate_hz', 1.0)
        
        # Initialize timer and publisher to None
        # These will be created in the lifecycle callbacks
        self._timer = None  # Timer for periodic health status publishing
        self._pub = None    # Publisher for the health status messages
        self._state_pub = None  # Publisher for operational state
        self._state_service = None  # Service for external state changes
        
        # Initialize operational state machine
        self._current_state = OperationalState.STARTUP
        self._state_history = []  # Track state transitions
        
        # Define valid state transitions (from_state -> [valid_to_states])
        self._valid_transitions = {
            OperationalState.STARTUP: [OperationalState.STANDBY, OperationalState.ERROR, OperationalState.EMERGENCY],
            OperationalState.STANDBY: [OperationalState.AUTONOMOUS, OperationalState.MANUAL, OperationalState.SHUTDOWN, OperationalState.EMERGENCY],
            OperationalState.AUTONOMOUS: [OperationalState.STANDBY, OperationalState.MANUAL, OperationalState.EMERGENCY, OperationalState.ERROR],
            OperationalState.MANUAL: [OperationalState.STANDBY, OperationalState.AUTONOMOUS, OperationalState.EMERGENCY, OperationalState.ERROR],
            OperationalState.EMERGENCY: [OperationalState.STANDBY],  # Can only return to standby after emergency
            OperationalState.ERROR: [OperationalState.STANDBY, OperationalState.SHUTDOWN],
            OperationalState.SHUTDOWN: []  # Terminal state
        }
        
        self.get_logger().info('System Manager node created (unconfigured state)')

    def on_configure(self, state: State):
        """
        Lifecycle callback: Unconfigured -> Inactive transition.
        
        This method is called when the node transitions from Unconfigured to Inactive state.
        Here we allocate resources like publishers, but don't start any active behavior yet.
        
        Args:
            state: The current lifecycle state
            
        Returns:
            TransitionCallbackReturn.SUCCESS if configuration succeeded
            TransitionCallbackReturn.FAILURE if configuration failed
        """
        self.get_logger().info('Configuring System Manager...')

        # Create the publisher for health status messages
        # Topic: '/system/health', Message Type: String, Queue Size: 10
        # Publisher is created here but won't publish until we're in Active state
        self._pub = self.create_publisher(String, '/system/health', 10)
        
        # Create publisher for operational state
        self._state_pub = self.create_publisher(OperationalState, '/system/state', 10)
        
        # Create service for external state changes
        self._state_service = self.create_service(
            ChangeState, 
            '/system/change_state', 
            self._handle_state_change_request
        )
        
        self.get_logger().info('State machine initialized. Current state: STARTUP')
        
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        """
        Lifecycle callback: Inactive -> Active transition.
        
        This method is called when the node transitions to Active state.
        Here we start the timer that triggers periodic health status publishing.
        The node is now fully operational.
        
        Args:
            state: The current lifecycle state
            
        Returns:
            TransitionCallbackReturn.SUCCESS if activation succeeded
            TransitionCallbackReturn.FAILURE if activation failed
        """
        self.get_logger().info('Activating System Manager...')

        # Get the configured health check rate parameter (in Hz)
        rate = self.get_parameter('health_rate_hz').value
        
        # Create a timer that calls _tick() at the specified rate
        # Timer period = 1.0 / rate (e.g., rate=1.0 Hz -> period=1.0 second)
        self._timer = self.create_timer(1.0 / float(rate), self._tick)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        """
        Lifecycle callback: Active -> Inactive transition.
        
        This method is called when the node transitions from Active to Inactive state.
        We stop the timer to cease publishing, but keep the publisher allocated.
        This allows quick reactivation without reallocating resources.
        
        Args:
            state: The current lifecycle state
            
        Returns:
            TransitionCallbackReturn.SUCCESS if deactivation succeeded
            TransitionCallbackReturn.FAILURE if deactivation failed
        """
        self.get_logger().info('Deactivating System Manager...')

        # Cancel the timer to stop periodic health status publishing
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: State):
        """
        Lifecycle callback: Any state -> Finalized transition.
        
        This method is called when the node is shutting down completely.
        We clean up all resources (timers, publishers) to ensure graceful shutdown.
        This can be called from any state, so we need to check what resources exist.
        
        Args:
            state: The current lifecycle state
            
        Returns:
            TransitionCallbackReturn.SUCCESS if shutdown succeeded
            TransitionCallbackReturn.FAILURE if shutdown failed
        """
        self.get_logger().info('Shutting down System Manager...')

        # Cancel and destroy the timer if it exists
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
            
        # Destroy the publisher if it exists
        if self._pub is not None:
            self._pub.destroy()
            self._pub = None

        return TransitionCallbackReturn.SUCCESS
    
    def _tick(self):
        """
        Timer callback function for periodic health status publishing.
        
        This method is called repeatedly by the timer at the configured rate
        (default: 1 Hz) when the node is in Active state. It creates and publishes
        a health status message.
        
        Note: This is a private method (prefixed with _) and should only be called
        by the internal timer, not directly by external code.
        """
        # Create a String message
        msg = String()
        msg.data = 'System is healthy'
        
        # Publish the health status message to '/system/health' topic
        self._pub.publish(msg)
        
        # Publish current operational state
        self._publish_state()
        
        # Log that we published (useful for debugging)
        self.get_logger().info('Published health status.')
    
    def _handle_state_change_request(self, request, response):
        """
        Service callback to handle external state change requests.
        
        Args:
            request: ChangeState.Request with requested_state and reason
            response: ChangeState.Response with success, current_state, and message
            
        Returns:
            response: Filled response object
        """
        requested_state = request.requested_state
        reason = request.reason if request.reason else "No reason provided"
        
        self.get_logger().info(f'State change requested: {self._state_name(requested_state)} (reason: {reason})')
        
        # Validate the state transition
        if self._is_valid_transition(requested_state):
            # Record the transition in history
            self._state_history.append({
                'from': self._current_state,
                'to': requested_state,
                'timestamp': self.get_clock().now(),
                'reason': reason
            })
            
            # Perform the state change
            old_state = self._current_state
            self._current_state = requested_state
            
            # Call state transition callback
            self._on_state_transition(old_state, requested_state)
            
            # Publish the new state immediately
            self._publish_state()
            
            # Fill successful response
            response.success = True
            response.current_state = self._current_state
            response.message = f'State changed from {self._state_name(old_state)} to {self._state_name(requested_state)}'
            
            self.get_logger().info(response.message)
        else:
            # Invalid transition - reject the request
            response.success = False
            response.current_state = self._current_state
            response.message = f'Invalid transition from {self._state_name(self._current_state)} to {self._state_name(requested_state)}'
            
            self.get_logger().warn(response.message)
        
        return response
    
    def _is_valid_transition(self, new_state):
        """
        Check if transitioning to new_state is valid from current state.
        
        Args:
            new_state: Requested target state
            
        Returns:
            bool: True if transition is valid, False otherwise
        """
        # Allow staying in the same state (no-op)
        if new_state == self._current_state:
            return True
        
        # Check if the transition is in the valid transitions map
        valid_states = self._valid_transitions.get(self._current_state, [])
        return new_state in valid_states
    
    def _on_state_transition(self, old_state, new_state):
        """
        Called when a state transition occurs. Override or extend for custom behavior.
        
        Args:
            old_state: Previous operational state
            new_state: New operational state
        """
        # Log the transition
        self.get_logger().info(f'State transition: {self._state_name(old_state)} -> {self._state_name(new_state)}')
        
        # Perform state-specific actions
        if new_state == OperationalState.EMERGENCY:
            self.get_logger().error('EMERGENCY STATE ACTIVATED!')
            # TODO: Trigger emergency stop procedures
            
        elif new_state == OperationalState.AUTONOMOUS:
            self.get_logger().info('Entering autonomous mode')
            # TODO: Enable autonomous systems
            
        elif new_state == OperationalState.MANUAL:
            self.get_logger().info('Entering manual control mode')
            # TODO: Enable teleoperation
            
        elif new_state == OperationalState.STANDBY:
            self.get_logger().info('Entering standby mode')
            # TODO: Put systems in standby
            
        elif new_state == OperationalState.SHUTDOWN:
            self.get_logger().info('Initiating shutdown sequence')
            # TODO: Graceful shutdown of all systems
    
    def _publish_state(self):
        """
        Publish the current operational state to /system/state topic.
        """
        if self._state_pub is None:
            return
        
        msg = OperationalState()
        msg.state = self._current_state
        msg.timestamp = self.get_clock().now().to_msg()
        msg.description = self._state_name(self._current_state)
        
        self._state_pub.publish(msg)
    
    def _state_name(self, state):
        """
        Get human-readable name for a state constant.
        
        Args:
            state: State constant value
            
        Returns:
            str: Human-readable state name
        """
        state_names = {
            OperationalState.STARTUP: 'STARTUP',
            OperationalState.STANDBY: 'STANDBY',
            OperationalState.AUTONOMOUS: 'AUTONOMOUS',
            OperationalState.MANUAL: 'MANUAL',
            OperationalState.EMERGENCY: 'EMERGENCY',
            OperationalState.ERROR: 'ERROR',
            OperationalState.SHUTDOWN: 'SHUTDOWN'
        }
        return state_names.get(state, f'UNKNOWN({state})')


def main():
    """
    Main entry point for the system_manager node.
    
    This function:
    1. Initializes the ROS2 Python client library
    2. Creates an instance of the SystemManager lifecycle node
    3. Spins the node to process callbacks (keeps it alive)
    4. Cleans up when the node is terminated (Ctrl+C or shutdown)
    
    Note: For lifecycle nodes, external lifecycle management commands are needed
    to transition the node through its states (configure, activate, etc.)
    """
    # Initialize the ROS2 Python client library
    rclpy.init()
    
    # Create the SystemManager node instance (starts in Unconfigured state)
    node = SystemManager()
    
    # Spin the node - this blocks and processes callbacks until shutdown
    # During spinning, the node responds to lifecycle transition commands
    rclpy.spin(node)
    
    # Clean up the node when spinning stops
    node.destroy_node()
    
    # Shutdown the ROS2 Python client library
    rclpy.shutdown()
