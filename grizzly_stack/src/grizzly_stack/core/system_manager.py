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
from lifecycle_msgs.srv import ChangeState as LifecycleChangeState, GetState
from lifecycle_msgs.msg import Transition


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
        
        # Lifecycle node management
        self._lifecycle_clients = {}  # Store lifecycle service clients for managed nodes
        self._managed_nodes = []  # List of nodes under lifecycle management
        self._node_states = {}  # Track the lifecycle state of each managed node
        
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
        
        This method manages lifecycle nodes based on operational state transitions,
        activating and deactivating subsystems as needed.
        
        Args:
            old_state: Previous operational state
            new_state: New operational state
        """
        # Log the transition
        self.get_logger().info(f'State transition: {self._state_name(old_state)} -> {self._state_name(new_state)}')
        
        # Perform state-specific actions and manage lifecycle nodes
        if new_state == OperationalState.EMERGENCY:
            self.get_logger().error('EMERGENCY STATE ACTIVATED!')
            # Deactivate all operational nodes immediately
            self._deactivate_lifecycle_nodes(['perception_node'])
            # TODO: Trigger emergency stop procedures for other subsystems
            
        elif new_state == OperationalState.AUTONOMOUS:
            self.get_logger().info('Entering autonomous mode - Activating perception')
            # Activate perception for autonomous navigation
            self._activate_lifecycle_nodes(['perception_node'])
            # TODO: Activate navigation, planning, and other autonomous subsystems
            
        elif new_state == OperationalState.MANUAL:
            self.get_logger().info('Entering manual control mode')
            # Perception can assist operator but not required
            # Keep perception active if coming from autonomous, or activate if coming from standby
            if old_state == OperationalState.STANDBY:
                self._activate_lifecycle_nodes(['perception_node'])
            # TODO: Activate teleoperation subsystems
            
        elif new_state == OperationalState.STANDBY:
            self.get_logger().info('Entering standby mode - Deactivating subsystems')
            # Deactivate operational nodes to save resources
            self._deactivate_lifecycle_nodes(['perception_node'])
            # TODO: Deactivate navigation, planning, and teleoperation subsystems
            
        elif new_state == OperationalState.SHUTDOWN:
            self.get_logger().info('Initiating shutdown sequence')
            # Deactivate all managed nodes
            self._deactivate_lifecycle_nodes(self._managed_nodes.copy())
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
    
    # === Lifecycle Node Management Methods ===
    
    def _get_lifecycle_client(self, node_name):
        """
        Get or create a lifecycle change state client for a managed node.
        
        Args:
            node_name: Name of the lifecycle node to manage
            
        Returns:
            Client object for lifecycle state transitions
        """
        if node_name not in self._lifecycle_clients:
            service_name = f'/{node_name}/change_state'
            self._lifecycle_clients[node_name] = self.create_client(
                LifecycleChangeState,
                service_name
            )
            self.get_logger().debug(f'Created lifecycle client for {node_name}')
        
        return self._lifecycle_clients[node_name]
    
    def _configure_lifecycle_node(self, node_name, timeout_sec=2.0):
        """
        Configure a lifecycle node (Unconfigured -> Inactive).
        
        This method makes an async call without blocking.
        
        Args:
            node_name: Name of the lifecycle node to configure
            timeout_sec: Timeout for service availability
        """
        client = self._get_lifecycle_client(node_name)
        
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f'Lifecycle service for {node_name} not available after {timeout_sec}s'
            )
            return
        
        request = LifecycleChangeState.Request()
        request.transition.id = Transition.TRANSITION_CONFIGURE
        
        self.get_logger().info(f'Requesting configuration of {node_name}...')
        
        # Make async call with callback
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self._handle_configure_response(f, node_name)
        )
    
    def _handle_configure_response(self, future, node_name):
        """Handle the response from lifecycle configuration request."""
        try:
            response = future.result()
            if response and response.success:
                self.get_logger().info(f'✅ Successfully configured {node_name}')
                self._node_states[node_name] = 'inactive'  # Track state
                # Now activate the node since configuration was triggered by activation request
                self.get_logger().info(f'Now activating {node_name}...')
                self._activate_lifecycle_node(node_name)
            else:
                self.get_logger().error(f'❌ Failed to configure {node_name}')
        except Exception as e:
            self.get_logger().error(f'Exception while configuring {node_name}: {e}')
    
    def _query_node_state(self, node_name, timeout_sec=2.0):
        """
        Query the actual current state of a lifecycle node.
        
        ⚠️ WARNING: This method is DEPRECATED and should NOT be used!
        
        This method uses rclpy.spin_until_future_complete() which causes DEADLOCK
        when called from within a service callback context (like _handle_state_change_request).
        The node cannot process the future because it's blocked waiting for itself.
        
        Instead, use the cached state in self._node_states which is maintained by
        lifecycle transition callbacks.
        
        Args:
            node_name: Name of the lifecycle node
            timeout_sec: Timeout for service call
            
        Returns:
            str: Current state label ('unconfigured', 'inactive', 'active', etc.)
                 or 'unknown' if query fails
        """
        # Create a GetState service client
        get_state_service = f'/{node_name}/get_state'
        client = self.create_client(GetState, get_state_service)
        
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f'GetState service for {node_name} not available after {timeout_sec}s'
            )
            return 'unknown'
        
        request = GetState.Request()
        future = client.call_async(request)
        
        # ⚠️ DEADLOCK WARNING: This will hang if called from a callback!
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        
        if future.result() is not None:
            state_label = future.result().current_state.label
            self.get_logger().debug(f'Queried state of {node_name}: {state_label}')
            # Update our cached state
            self._node_states[node_name] = state_label
            return state_label
        else:
            self.get_logger().warn(f'Failed to query state of {node_name}')
            return 'unknown'
    
    def _activate_lifecycle_node(self, node_name, timeout_sec=2.0):
        """
        Activate a lifecycle node (Inactive -> Active).
        
        This method makes an async call without blocking. The result is handled
        via callback to avoid deadlock issues.
        
        Note: We rely on the cached state from lifecycle_manager initialization.
        If the node is unconfigured, we configure it first. The lifecycle system
        will reject invalid transitions, so we don't need to query state here
        (which would cause deadlock when called from a callback context).
        
        Args:
            node_name: Name of the lifecycle node to activate
            timeout_sec: Timeout for service availability check
        """
        # Use cached state if available (populated by lifecycle_manager)
        cached_state = self._node_states.get(node_name, 'inactive')
        
        # If cached state shows already active, skip
        if cached_state == 'active':
            self.get_logger().info(
                f'Skipping activation of {node_name} - cached state shows already active'
            )
            return
        
        # If node is unconfigured, configure it first
        if cached_state == 'unconfigured':
            self.get_logger().info(
                f'{node_name} is in unconfigured state - configuring first'
            )
            self._configure_lifecycle_node(node_name, timeout_sec)
            # Note: The configure callback will trigger activation
            return
        
        client = self._get_lifecycle_client(node_name)
        
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f'Lifecycle service for {node_name} not available after {timeout_sec}s'
            )
            return
        
        request = LifecycleChangeState.Request()
        request.transition.id = Transition.TRANSITION_ACTIVATE
        
        self.get_logger().info(f'Requesting activation of {node_name}...')
        
        # Make async call with callback
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self._handle_activate_response(f, node_name)
        )
    
    def _handle_activate_response(self, future, node_name):
        """Handle the response from lifecycle activation request."""
        try:
            response = future.result()
            if response and response.success:
                self.get_logger().info(f'✅ Successfully activated {node_name}')
                self._node_states[node_name] = 'active'  # Track state
                if node_name not in self._managed_nodes:
                    self._managed_nodes.append(node_name)
            else:
                self.get_logger().error(f'❌ Failed to activate {node_name}')
        except Exception as e:
            self.get_logger().error(f'Exception while activating {node_name}: {e}')
    
    def _configure_lifecycle_node(self, node_name, timeout_sec=2.0):
        """
        Configure a lifecycle node (Unconfigured -> Inactive).
        
        This method makes an async call without blocking. After successful configuration,
        it will automatically trigger activation if that was the original intent.
        
        Args:
            node_name: Name of the lifecycle node to configure
            timeout_sec: Timeout for service availability check
        """
        client = self._get_lifecycle_client(node_name)
        
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f'Lifecycle service for {node_name} not available after {timeout_sec}s'
            )
            return
        
        request = LifecycleChangeState.Request()
        request.transition.id = Transition.TRANSITION_CONFIGURE
        
        self.get_logger().info(f'Requesting configuration of {node_name}...')
        
        # Make async call with callback
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self._handle_configure_response(f, node_name)
        )
    
    def _handle_configure_response(self, future, node_name):
        """Handle the response from lifecycle configuration request."""
        try:
            response = future.result()
            if response and response.success:
                self.get_logger().info(f'✅ Successfully configured {node_name}')
                self._node_states[node_name] = 'inactive'  # Track state
                # Now activate the node since configuration was triggered by activation request
                self.get_logger().info(f'Now activating {node_name}...')
                self._activate_lifecycle_node(node_name)
            else:
                self.get_logger().error(f'❌ Failed to configure {node_name}')
        except Exception as e:
            self.get_logger().error(f'Exception while configuring {node_name}: {e}')
    
    def _deactivate_lifecycle_node(self, node_name, timeout_sec=2.0):
        """
        Deactivate a lifecycle node (Active -> Inactive).
        
        This method makes an async call without blocking. We use cached state
        instead of querying to avoid deadlock when called from callback context.
        The lifecycle system will reject invalid transitions if needed.
        
        Args:
            node_name: Name of the lifecycle node to deactivate
            timeout_sec: Timeout for service availability check
        """
        # Use cached state to avoid deadlock
        # Default to 'inactive' since lifecycle_manager configures nodes to inactive state
        cached_state = self._node_states.get(node_name, 'inactive')
        
        if cached_state != 'active':
            self.get_logger().info(
                f'Skipping deactivation of {node_name} - cached state is {cached_state} (not active)'
            )
            return
        
        client = self._get_lifecycle_client(node_name)
        
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f'Lifecycle service for {node_name} not available after {timeout_sec}s'
            )
            return
        
        request = LifecycleChangeState.Request()
        request.transition.id = Transition.TRANSITION_DEACTIVATE
        
        self.get_logger().info(f'Requesting deactivation of {node_name}...')
        
        # Make async call with callback
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self._handle_deactivate_response(f, node_name)
        )
    
    def _handle_deactivate_response(self, future, node_name):
        """Handle the response from lifecycle deactivation request."""
        try:
            response = future.result()
            if response and response.success:
                self.get_logger().info(f'✅ Successfully deactivated {node_name}')
                self._node_states[node_name] = 'inactive'  # Track state
            else:
                self.get_logger().error(f'❌ Failed to deactivate {node_name}')
        except Exception as e:
            self.get_logger().error(f'Exception while deactivating {node_name}: {e}')
    
    def _cleanup_lifecycle_node(self, node_name, timeout_sec=2.0):
        """
        Cleanup a lifecycle node (Inactive -> Unconfigured).
        
        This method makes an async call without blocking.
        
        Args:
            node_name: Name of the lifecycle node to cleanup
            timeout_sec: Timeout for service availability
        """
        client = self._get_lifecycle_client(node_name)
        
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f'Lifecycle service for {node_name} not available after {timeout_sec}s'
            )
            return
        
        request = LifecycleChangeState.Request()
        request.transition.id = Transition.TRANSITION_CLEANUP
        
        self.get_logger().info(f'Requesting cleanup of {node_name}...')
        
        # Make async call with callback
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self._handle_cleanup_response(f, node_name)
        )
    
    def _handle_cleanup_response(self, future, node_name):
        """Handle the response from lifecycle cleanup request."""
        try:
            response = future.result()
            if response and response.success:
                self.get_logger().info(f'✅ Successfully cleaned up {node_name}')
                if node_name in self._managed_nodes:
                    self._managed_nodes.remove(node_name)
            else:
                self.get_logger().error(f'❌ Failed to cleanup {node_name}')
        except Exception as e:
            self.get_logger().error(f'Exception while cleaning up {node_name}: {e}')
    
    def _activate_lifecycle_nodes(self, node_names):
        """
        Activate multiple lifecycle nodes.
        
        Args:
            node_names: List of node names to activate
        """
        for node_name in node_names:
            self._activate_lifecycle_node(node_name)
    
    def _deactivate_lifecycle_nodes(self, node_names):
        """
        Deactivate multiple lifecycle nodes.
        
        Args:
            node_names: List of node names to deactivate
        """
        for node_name in node_names:
            self._deactivate_lifecycle_node(node_name)


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
