"""
Base Lifecycle Node for Grizzly Robotics Stack

This module provides a base class for lifecycle-managed nodes that eliminates
boilerplate code and provides sensible defaults. New nodes can inherit from
GrizzlyLifecycleNode and only override the methods they need.

Example Usage:
    class MyNode(GrizzlyLifecycleNode):
        def __init__(self):
            super().__init__('my_node')
            
            # Declare your parameters
            self.declare_parameter('my_param', 'default_value')
        
        def on_configure_hook(self) -> bool:
            # Your configuration logic here
            # Create publishers, load params, etc.
            return True
        
        def on_activate_hook(self) -> bool:
            # Start your timers, processing, etc.
            return True
        
        def tick(self):
            # Called periodically when active (if rate_hz > 0)
            pass
"""

import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn, State
from std_msgs.msg import String
from typing import Optional, List, Any


class GrizzlyLifecycleNode(LifecycleNode):
    """
    Base class for all Grizzly lifecycle-managed nodes.
    
    This class handles common lifecycle patterns:
    - Automatic timer creation/destruction based on rate parameter
    - Standard status publishing
    - Enabled/disabled configuration
    - Graceful error handling in all lifecycle callbacks
    
    Subclasses should override the *_hook methods instead of the on_* methods
    to add their specific logic while keeping the base lifecycle management.
    
    Lifecycle States:
    - Unconfigured: Initial state, no resources allocated
    - Inactive: Configured but not processing
    - Active: Fully operational
    - Finalized: Shutdown complete
    """
    
    def __init__(self, node_name: str, **kwargs):
        """
        Initialize the lifecycle node.
        
        Args:
            node_name: Name of this ROS node
            **kwargs: Additional arguments passed to LifecycleNode
        """
        super().__init__(node_name, **kwargs)
        
        # Common parameters for all nodes
        self.declare_parameter('enabled', True)
        self.declare_parameter('rate_hz', 1.0)  # Main loop rate, 0 = no timer
        
        # Internal state
        self._timer = None
        self._status_pub = None
        self._enabled = True
        self._rate_hz = 1.0
        self._tick_count = 0
        
        # Status publishing config
        self._status_topic = f'/{node_name}/status'
        
        self.get_logger().info(f'{node_name} created (unconfigured state)')
    
    # =========================================================================
    # Lifecycle Callbacks - Override *_hook methods, not these
    # =========================================================================
    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Lifecycle callback: Unconfigured -> Inactive transition.
        
        Do not override this method. Override on_configure_hook() instead.
        """
        self.get_logger().info(f'Configuring {self.get_name()}...')
        
        try:
            # Load common parameters
            self._enabled = self.get_parameter('enabled').value
            self._rate_hz = self.get_parameter('rate_hz').value
            
            if not self._enabled:
                self.get_logger().warn(
                    f'{self.get_name()} is disabled in configuration'
                )
            
            # Create status publisher
            self._status_pub = self.create_publisher(
                String,
                self._status_topic,
                10
            )
            
            # Call subclass hook
            if not self.on_configure_hook():
                return TransitionCallbackReturn.FAILURE
            
            self.get_logger().info(
                f'{self.get_name()} configured: enabled={self._enabled}, '
                f'rate={self._rate_hz}Hz'
            )
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to configure {self.get_name()}: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """
        Lifecycle callback: Inactive -> Active transition.
        
        Do not override this method. Override on_activate_hook() instead.
        """
        self.get_logger().info(f'Activating {self.get_name()}...')
        
        try:
            if not self._enabled:
                self.get_logger().warn(
                    f'Cannot activate: {self.get_name()} is disabled'
                )
                return TransitionCallbackReturn.FAILURE
            
            # Call subclass hook first
            if not self.on_activate_hook():
                return TransitionCallbackReturn.FAILURE
            
            # Start timer if rate is configured
            if self._rate_hz > 0:
                timer_period = 1.0 / self._rate_hz
                self._timer = self.create_timer(timer_period, self._timer_callback)
            
            # Reset tick counter
            self._tick_count = 0
            
            # Publish activation status
            self.publish_status('ACTIVE')
            
            self.get_logger().info(f'{self.get_name()} activated')
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to activate {self.get_name()}: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """
        Lifecycle callback: Active -> Inactive transition.
        
        Do not override this method. Override on_deactivate_hook() instead.
        """
        self.get_logger().info(f'Deactivating {self.get_name()}...')
        
        try:
            # Stop timer
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
            
            # Call subclass hook
            if not self.on_deactivate_hook():
                return TransitionCallbackReturn.FAILURE
            
            # Publish deactivation status
            self.publish_status(f'INACTIVE - Processed {self._tick_count} ticks')
            
            self.get_logger().info(
                f'{self.get_name()} deactivated after {self._tick_count} ticks'
            )
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to deactivate {self.get_name()}: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Lifecycle callback: Inactive -> Unconfigured transition.
        
        Do not override this method. Override on_cleanup_hook() instead.
        """
        self.get_logger().info(f'Cleaning up {self.get_name()}...')
        
        try:
            # Call subclass hook first
            if not self.on_cleanup_hook():
                return TransitionCallbackReturn.FAILURE
            
            # Destroy status publisher
            if self._status_pub is not None:
                self.destroy_publisher(self._status_pub)
                self._status_pub = None
            
            self._timer = None
            
            self.get_logger().info(f'{self.get_name()} cleaned up')
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to cleanup {self.get_name()}: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Lifecycle callback: Any state -> Finalized transition.
        
        Do not override this method. Override on_shutdown_hook() instead.
        """
        self.get_logger().info(f'Shutting down {self.get_name()}...')
        
        try:
            # Stop timer if running
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
            
            # Call subclass hook
            if not self.on_shutdown_hook():
                return TransitionCallbackReturn.FAILURE
            
            # Destroy status publisher
            if self._status_pub is not None:
                self.destroy_publisher(self._status_pub)
                self._status_pub = None
            
            self.get_logger().info(f'{self.get_name()} shutdown complete')
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to shutdown {self.get_name()}: {e}')
            return TransitionCallbackReturn.FAILURE
    
    # =========================================================================
    # Hook Methods - Override these in subclasses
    # =========================================================================
    
    def on_configure_hook(self) -> bool:
        """
        Called during configuration. Override to add your configuration logic.
        
        This is where you should:
        - Read parameters
        - Create publishers and subscribers
        - Initialize data structures
        
        Returns:
            True if configuration succeeded, False otherwise
        """
        return True
    
    def on_activate_hook(self) -> bool:
        """
        Called during activation. Override to add your activation logic.
        
        This is where you should:
        - Start any additional timers
        - Initialize state for active operation
        - Perform any pre-activation checks
        
        Returns:
            True if activation succeeded, False otherwise
        """
        return True
    
    def on_deactivate_hook(self) -> bool:
        """
        Called during deactivation. Override to add your deactivation logic.
        
        This is where you should:
        - Stop any additional timers
        - Send stop commands (e.g., zero velocity)
        - Clean up active state
        
        Returns:
            True if deactivation succeeded, False otherwise
        """
        return True
    
    def on_cleanup_hook(self) -> bool:
        """
        Called during cleanup. Override to add your cleanup logic.
        
        This is where you should:
        - Destroy any publishers/subscribers you created
        - Release any resources
        
        Returns:
            True if cleanup succeeded, False otherwise
        """
        return True
    
    def on_shutdown_hook(self) -> bool:
        """
        Called during shutdown. Override to add your shutdown logic.
        
        This is where you should:
        - Emergency stop procedures
        - Clean up any resources that might still exist
        
        Returns:
            True if shutdown succeeded, False otherwise
        """
        return True
    
    def tick(self):
        """
        Called periodically when the node is active.
        
        Override this method to implement your main processing loop.
        This is called at the rate specified by the 'rate_hz' parameter.
        """
        pass
    
    # =========================================================================
    # Utility Methods
    # =========================================================================
    
    def _timer_callback(self):
        """Internal timer callback that calls tick() and updates counter."""
        self._tick_count += 1
        self.tick()
    
    def publish_status(self, status_text: str):
        """
        Publish a status message to the node's status topic.
        
        Args:
            status_text: Status message string
        """
        if self._status_pub is None:
            return
        
        msg = String()
        msg.data = f'[{self.get_clock().now().nanoseconds / 1e9:.2f}] {status_text}'
        self._status_pub.publish(msg)
        
        self.get_logger().debug(f'Status: {status_text}')
    
    @property
    def enabled(self) -> bool:
        """Whether this node is enabled."""
        return self._enabled
    
    @property
    def tick_count(self) -> int:
        """Number of ticks processed since activation."""
        return self._tick_count


def run_node(node_class, args=None):
    """
    Convenience function to run a lifecycle node.
    
    This handles ROS initialization, spinning, and cleanup.
    
    Args:
        node_class: The node class to instantiate and run
        args: Optional command line arguments
        
    Example:
        if __name__ == '__main__':
            run_node(MyNode)
    """
    rclpy.init(args=args)
    
    node = node_class()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
