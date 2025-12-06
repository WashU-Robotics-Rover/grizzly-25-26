"""
System Manager Node for Grizzly Robotics Stack

This module implements a ROS2 Lifecycle Node that monitors and reports system health.
Lifecycle nodes provide deterministic state management with defined transitions between
states (Unconfigured -> Inactive -> Active -> etc.)
"""

import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn, State
from std_msgs.msg import String
from grizzly_interfaces.msg import OperationalState
from grizzly_interfaces.srv import ChangeState

from .layer_manager import LayerManager
from .utils import get_state_name, get_valid_transitions


class SystemManager(LifecycleNode):
    """
    SystemManager is a ROS2 Lifecycle Node that manages system health reporting.
    
    Lifecycle States:
    - Unconfigured: Initial state, no resources allocated
    - Inactive: Configured but not running
    - Active: Fully operational, publishing health status
    - Finalized: Shutdown complete
    
    This node publishes periodic health status messages to '/system/health' topic
    when in the Active state.
    """
    
    def __init__(self):
        """Initialize the SystemManager node in the Unconfigured state."""
        super().__init__('system_manager')
        
        # Declare parameters
        self.declare_parameter('health_rate_hz', 1.0)
        
        # Initialize resources to None
        self._timer = None
        self._pub = None
        self._state_pub = None
        self._state_service = None
        
        # Initialize operational state machine
        self._current_state = OperationalState.STARTUP
        self._state_history = []
        
        # Initialize layer manager for managing nodes by layer
        self._layer_manager = LayerManager(self)
        
        # Track node states for layer manager initialization
        self._node_states = {}
        
        # Valid state transitions
        self._valid_transitions = get_valid_transitions()
        
        self.get_logger().info('System Manager node created (unconfigured state)')

    def on_configure(self, state: State):
        """Lifecycle callback: Unconfigured -> Inactive transition."""
        self.get_logger().info('Configuring System Manager...')

        # Create publishers
        self._pub = self.create_publisher(String, '/system/health', 10)
        self._state_pub = self.create_publisher(OperationalState, '/system/state', 10)
        
        # Create service for external state changes
        self._state_service = self.create_service(
            ChangeState, 
            '/system/change_state', 
            self._handle_state_change_request
        )
        
        self.get_logger().info('State machine initialized. Current state: STARTUP')
        
        # Initialize layer manager with node states
        initial_node_states = {}
        for layer_name in self._layer_manager.get_all_layers():
            for node_name in self._layer_manager.get_layer_nodes(layer_name):
                initial_node_states[node_name] = 'inactive'
        
        self._layer_manager.initialize_node_states(initial_node_states)
        self._node_states.update(initial_node_states)
        
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        """Lifecycle callback: Inactive -> Active transition."""
        self.get_logger().info('Activating System Manager...')

        rate = self.get_parameter('health_rate_hz').value
        self._timer = self.create_timer(1.0 / float(rate), self._tick)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        """Lifecycle callback: Active -> Inactive transition."""
        self.get_logger().info('Deactivating System Manager...')

        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: State):
        """Lifecycle callback: Any state -> Finalized transition."""
        self.get_logger().info('Shutting down System Manager...')

        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
            
        if self._pub is not None:
            self._pub.destroy()
            self._pub = None

        return TransitionCallbackReturn.SUCCESS
    
    def _tick(self):
        """Timer callback for periodic health status publishing."""
        msg = String()
        msg.data = 'System is healthy'
        self._pub.publish(msg)
        self._publish_state()
        self.get_logger().info('Published health status.')
    
    def _handle_state_change_request(self, request, response):
        """Service callback to handle external state change requests."""
        requested_state = request.requested_state
        reason = request.reason if request.reason else "No reason provided"
        
        self.get_logger().info(
            f'State change requested: {get_state_name(requested_state)} (reason: {reason})'
        )
        
        if self._is_valid_transition(requested_state):
            # Record the transition
            self._state_history.append({
                'from': self._current_state,
                'to': requested_state,
                'timestamp': self.get_clock().now(),
                'reason': reason
            })
            
            old_state = self._current_state
            self._current_state = requested_state
            
            # Delegate layer management to layer manager
            self._layer_manager.handle_state_transition(old_state, requested_state)
            
            self._publish_state()
            
            response.success = True
            response.current_state = self._current_state
            response.message = (
                f'State changed from {get_state_name(old_state)} '
                f'to {get_state_name(requested_state)}'
            )
            
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.current_state = self._current_state
            response.message = (
                f'Invalid transition from {get_state_name(self._current_state)} '
                f'to {get_state_name(requested_state)}'
            )
            
            self.get_logger().warn(response.message)
        
        return response
    
    def _is_valid_transition(self, new_state):
        """Check if transitioning to new_state is valid from current state."""
        if new_state == self._current_state:
            return True
        
        valid_states = self._valid_transitions.get(self._current_state, [])
        return new_state in valid_states
    
    def _publish_state(self):
        """Publish the current operational state."""
        if self._state_pub is None:
            return
        
        msg = OperationalState()
        msg.state = self._current_state
        msg.timestamp = self.get_clock().now().to_msg()
        msg.description = get_state_name(self._current_state)
        
        self._state_pub.publish(msg)


def main():
    """Main entry point for the system_manager node."""
    rclpy.init()
    node = SystemManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
