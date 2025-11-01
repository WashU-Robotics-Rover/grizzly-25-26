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
        
        # Log that we published (useful for debugging)
        self.get_logger().info('Published health status.')


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
