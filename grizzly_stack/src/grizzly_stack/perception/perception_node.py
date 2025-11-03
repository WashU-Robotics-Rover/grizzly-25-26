"""
Test Perception Node for Grizzly Robotics Stack

This module implements a lifecycle-managed perception node that serves as a working
template for real-world perception implementations. It simulates sensor data processing
and demonstrates proper lifecycle state management.

This node can be enabled/disabled via configuration and responds to lifecycle transitions
from the System Manager for coordinated state-based activation.
"""

import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn, State
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
import random


class PerceptionNode(LifecycleNode):
    """
    Test Perception Node - A lifecycle-managed perception template.
    
    Lifecycle States:
    - Unconfigured: Initial state, no resources allocated
    - Inactive: Configured but not processing (resources allocated but idle)
    - Active: Fully operational, processing sensor data
    - Finalized: Shutdown complete
    
    When active, this node:
    - Simulates object detection at configured rate
    - Publishes detection results
    - Reports perception status
    - Can be dynamically activated/deactivated by System Manager
    """
    
    def __init__(self):
        """
        Initialize the PerceptionNode in the Unconfigured state.
        
        At this stage, we only declare parameters and initialize member variables.
        No active resources (publishers, timers) are created yet.
        """
        super().__init__('perception_node')
        
        # Declare parameters with defaults
        self.declare_parameter('enabled', True)  # Whether perception should run
        self.declare_parameter('detection_rate_hz', 2.0)  # Detection frequency
        self.declare_parameter('confidence_threshold', 0.5)  # Detection confidence
        self.declare_parameter('max_detection_range_m', 10.0)  # Max detection distance
        self.declare_parameter('sensor_type', 'camera')  # Sensor type: camera, lidar, etc.
        
        # Initialize resources to None (created in lifecycle callbacks)
        self._timer = None
        self._detection_pub = None
        self._status_pub = None
        
        # State tracking
        self._detection_count = 0
        self._last_detection_time = None
        
        self.get_logger().info('Perception node created (unconfigured state)')
    
    def on_configure(self, state: State):
        """
        Lifecycle callback: Unconfigured -> Inactive transition.
        
        Allocate resources like publishers and load configuration, but don't
        start active processing yet.
        
        Args:
            state: The current lifecycle state
            
        Returns:
            TransitionCallbackReturn.SUCCESS if configuration succeeded
            TransitionCallbackReturn.FAILURE if configuration failed
        """
        self.get_logger().info('Configuring Perception Node...')
        
        try:
            # Get parameters
            self.enabled = self.get_parameter('enabled').value
            self.detection_rate = self.get_parameter('detection_rate_hz').value
            self.confidence_threshold = self.get_parameter('confidence_threshold').value
            self.max_range = self.get_parameter('max_detection_range_m').value
            self.sensor_type = self.get_parameter('sensor_type').value
            
            # Check if node should be enabled
            if not self.enabled:
                self.get_logger().warn('Perception node is disabled in configuration')
                # Still configure successfully, just won't activate
            
            # Create publishers (but don't start publishing yet)
            self._detection_pub = self.create_publisher(
                String,
                '/perception/detections',
                10
            )
            
            self._status_pub = self.create_publisher(
                String,
                '/perception/status',
                10
            )
            
            self.get_logger().info(
                f'Perception configured: sensor={self.sensor_type}, '
                f'rate={self.detection_rate}Hz, enabled={self.enabled}'
            )
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to configure perception node: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: State):
        """
        Lifecycle callback: Inactive -> Active transition.
        
        Start active perception processing. This is called when the node should
        begin its operational duties.
        
        Args:
            state: The current lifecycle state
            
        Returns:
            TransitionCallbackReturn.SUCCESS if activation succeeded
            TransitionCallbackReturn.FAILURE if activation failed
        """
        self.get_logger().info('Activating Perception Node...')
        
        try:
            # Check if enabled
            if not self.enabled:
                self.get_logger().warn('Cannot activate: perception node is disabled')
                return TransitionCallbackReturn.FAILURE
            
            # Start the detection timer
            timer_period = 1.0 / self.detection_rate  # Convert Hz to seconds
            self._timer = self.create_timer(timer_period, self._detection_callback)
            
            # Reset counters
            self._detection_count = 0
            self._last_detection_time = self.get_clock().now()
            
            # Publish activation status
            self._publish_status('ACTIVE - Processing sensor data')
            
            self.get_logger().info(
                f'Perception node activated: Running {self.sensor_type} detection '
                f'at {self.detection_rate}Hz'
            )
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to activate perception node: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def on_deactivate(self, state: State):
        """
        Lifecycle callback: Active -> Inactive transition.
        
        Stop active processing but keep resources allocated. This allows quick
        reactivation without full reconfiguration.
        
        Args:
            state: The current lifecycle state
            
        Returns:
            TransitionCallbackReturn.SUCCESS if deactivation succeeded
            TransitionCallbackReturn.FAILURE if deactivation failed
        """
        self.get_logger().info('Deactivating Perception Node...')
        
        try:
            # Stop the detection timer
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
            
            # Publish deactivation status
            self._publish_status(
                f'INACTIVE - Processed {self._detection_count} detections'
            )
            
            self.get_logger().info(
                f'Perception node deactivated after {self._detection_count} detections'
            )
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to deactivate perception node: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def on_cleanup(self, state: State):
        """
        Lifecycle callback: Inactive -> Unconfigured transition.
        
        Release all allocated resources. This is the inverse of configure.
        
        Args:
            state: The current lifecycle state
            
        Returns:
            TransitionCallbackReturn.SUCCESS if cleanup succeeded
            TransitionCallbackReturn.FAILURE if cleanup failed
        """
        self.get_logger().info('Cleaning up Perception Node...')
        
        try:
            # Destroy publishers
            if self._detection_pub is not None:
                self.destroy_publisher(self._detection_pub)
                self._detection_pub = None
            
            if self._status_pub is not None:
                self.destroy_publisher(self._status_pub)
                self._status_pub = None
            
            # Clear timer (should already be None from deactivate)
            self._timer = None
            
            self.get_logger().info('Perception node cleaned up')
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to cleanup perception node: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def on_shutdown(self, state: State):
        """
        Lifecycle callback: Any state -> Finalized transition.
        
        Emergency shutdown from any state. Clean up any resources that might exist.
        
        Args:
            state: The current lifecycle state
            
        Returns:
            TransitionCallbackReturn.SUCCESS if shutdown succeeded
            TransitionCallbackReturn.FAILURE if shutdown failed
        """
        self.get_logger().info('Shutting down Perception Node...')
        
        try:
            # Stop timer if running
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
            
            # Destroy publishers if they exist
            if self._detection_pub is not None:
                self.destroy_publisher(self._detection_pub)
                self._detection_pub = None
            
            if self._status_pub is not None:
                self.destroy_publisher(self._status_pub)
                self._status_pub = None
            
            self.get_logger().info('Perception node shutdown complete')
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to shutdown perception node: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def _detection_callback(self):
        """
        Timer callback that simulates object detection processing.
        
        This method is called periodically when the node is Active. It simulates
        processing sensor data and detecting objects.
        """
        # Simulate object detection
        num_objects = random.randint(0, 5)
        detections = []
        
        for i in range(num_objects):
            # Simulate detection data
            detection = {
                'id': f'obj_{self._detection_count}_{i}',
                'confidence': random.uniform(self.confidence_threshold, 1.0),
                'distance': random.uniform(0.5, self.max_range),
                'class': random.choice(['rock', 'crater', 'marker', 'obstacle']),
                'x': random.uniform(-5.0, 5.0),
                'y': random.uniform(-5.0, 5.0),
                'z': random.uniform(-1.0, 1.0)
            }
            
            # Only keep detections above confidence threshold
            if detection['confidence'] >= self.confidence_threshold:
                detections.append(detection)
        
        # Publish detections
        if detections:
            self._publish_detections(detections)
        
        # Update counter
        self._detection_count += 1
        
        # Periodically publish status
        if self._detection_count % 10 == 0:
            self._publish_status(
                f'ACTIVE - Processed {self._detection_count} frames, '
                f'{len(detections)} objects detected'
            )
    
    def _publish_detections(self, detections):
        """
        Publish detection results.
        
        Args:
            detections: List of detection dictionaries
        """
        if self._detection_pub is None:
            return
        
        # Create a simple string message with detection info
        # In a real implementation, use custom messages or standard types
        detection_str = f"[Frame {self._detection_count}] Detected {len(detections)} objects:\n"
        
        for det in detections:
            detection_str += (
                f"  - {det['class']}: "
                f"pos=({det['x']:.2f}, {det['y']:.2f}, {det['z']:.2f}), "
                f"dist={det['distance']:.2f}m, "
                f"conf={det['confidence']:.2f}\n"
            )
        
        msg = String()
        msg.data = detection_str
        self._detection_pub.publish(msg)
        
        self.get_logger().debug(f'Published {len(detections)} detections')
    
    def _publish_status(self, status_text):
        """
        Publish perception status message.
        
        Args:
            status_text: Status message string
        """
        if self._status_pub is None:
            return
        
        msg = String()
        msg.data = f'[{self.get_clock().now().nanoseconds / 1e9:.2f}] {status_text}'
        self._status_pub.publish(msg)
        
        self.get_logger().info(f'Status: {status_text}')


def main(args=None):
    """
    Main entry point for the perception_node.
    
    This function:
    1. Initializes the ROS2 Python client library
    2. Creates an instance of the PerceptionNode lifecycle node
    3. Spins the node to process callbacks
    4. Cleans up when the node is terminated
    
    Note: As a lifecycle node, external lifecycle management commands are needed
    to transition the node through its states (configure, activate, etc.)
    """
    rclpy.init(args=args)
    
    # Create the PerceptionNode instance (starts in Unconfigured state)
    node = PerceptionNode()
    
    # Spin the node - processes callbacks until shutdown
    # During spinning, the node responds to lifecycle transition commands
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Clean up the node when spinning stops
    node.destroy_node()
    
    # Shutdown the ROS2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
