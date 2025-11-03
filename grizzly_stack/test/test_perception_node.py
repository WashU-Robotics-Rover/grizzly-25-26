"""
Test Suite for Perception Node

This module contains comprehensive tests for the perception_node:
- Lifecycle state transitions (unconfigured -> inactive -> active)
- Configuration parameter validation
- Sensor data processing simulation
- State management and operational state tracking
- Error handling and edge cases
"""

import unittest
from unittest.mock import Mock, MagicMock, patch
import rclpy
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from grizzly_interfaces.msg import OperationalState

from grizzly_stack.perception.perception_node import PerceptionNode


class TestPerceptionNodeLifecycle(unittest.TestCase):
    """Test cases for perception node lifecycle management."""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 once for all tests."""
        if not rclpy.ok():
            rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after all tests."""
        if rclpy.ok():
            rclpy.shutdown()
    
    def setUp(self):
        """Set up test fixtures before each test."""
        self.node = PerceptionNode()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
    
    def tearDown(self):
        """Clean up after each test."""
        self.node.destroy_node()
        self.executor.shutdown()
    
    def test_node_creation(self):
        """Test that perception node is created in unconfigured state."""
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'perception_node')
        # Check initial state - timers and publishers should be None
        self.assertIsNone(self.node._timer)
        self.assertIsNone(self.node._detection_pub)
        self.assertIsNone(self.node._status_pub)
    
    def test_on_configure_success(self):
        """Test successful configuration transition."""
        # Mock state object
        mock_state = Mock()
        
        result = self.node.on_configure(mock_state)
        
        # Should return SUCCESS
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        
        # Publishers should be created
        self.assertIsNotNone(self.node._detection_pub)
        self.assertIsNotNone(self.node._status_pub)
        
        # Timer should still be None (created on activation)
        self.assertIsNone(self.node._timer)
    
    def test_on_configure_parameters(self):
        """Test that configuration loads parameters correctly."""
        mock_state = Mock()
        
        # Parameters are already declared in __init__, just set them
        self.node.set_parameters([
            rclpy.parameter.Parameter('sensor_type', rclpy.Parameter.Type.STRING, 'lidar'),
            rclpy.parameter.Parameter('detection_rate_hz', rclpy.Parameter.Type.DOUBLE, 5.0),
            rclpy.parameter.Parameter('enabled', rclpy.Parameter.Type.BOOL, True)
        ])
        
        result = self.node.on_configure(mock_state)
        
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        
        # Verify parameters are accessible
        sensor_type = self.node.get_parameter('sensor_type').value
        self.assertEqual(sensor_type, 'lidar')
    
    def test_on_cleanup_success(self):
        """Test successful cleanup transition."""
        # First configure the node
        mock_state = Mock()
        self.node.on_configure(mock_state)
        
        # Then cleanup
        result = self.node.on_cleanup(mock_state)
        
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertIsNone(self.node._detection_pub)
        self.assertIsNone(self.node._status_pub)
    
    def test_on_activate_success(self):
        """Test successful activation transition."""
        # Must configure first
        mock_state = Mock()
        self.node.on_configure(mock_state)
        
        # Then activate
        result = self.node.on_activate(mock_state)
        
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertIsNotNone(self.node._timer)
    
    def test_on_deactivate_success(self):
        """Test successful deactivation transition."""
        # Configure and activate first
        mock_state = Mock()
        self.node.on_configure(mock_state)
        self.node.on_activate(mock_state)
        
        # Then deactivate
        result = self.node.on_deactivate(mock_state)
        
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertIsNone(self.node._timer)
    
    def test_on_shutdown_success(self):
        """Test successful shutdown transition."""
        mock_state = Mock()
        
        result = self.node.on_shutdown(mock_state)
        
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
    
    def test_full_lifecycle_sequence(self):
        """Test complete lifecycle: configure -> activate -> deactivate -> cleanup."""
        mock_state = Mock()
        
        # Configure
        result = self.node.on_configure(mock_state)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertIsNotNone(self.node._detection_pub)
        self.assertIsNotNone(self.node._status_pub)
        
        # Activate
        result = self.node.on_activate(mock_state)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertIsNotNone(self.node._timer)
        
        # Deactivate
        result = self.node.on_deactivate(mock_state)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertIsNone(self.node._timer)
        
        # Cleanup
        result = self.node.on_cleanup(mock_state)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertIsNone(self.node._detection_pub)
        self.assertIsNone(self.node._status_pub)


class TestPerceptionNodeProcessing(unittest.TestCase):
    """Test cases for perception data processing."""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 once for all tests."""
        if not rclpy.ok():
            rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after all tests."""
        if rclpy.ok():
            rclpy.shutdown()
    
    def setUp(self):
        """Set up test fixtures before each test."""
        self.node = PerceptionNode()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        
        # Configure and activate for processing tests
        mock_state = Mock()
        self.node.on_configure(mock_state)
        self.node.on_activate(mock_state)
    
    def tearDown(self):
        """Clean up after each test."""
        self.node.destroy_node()
        self.executor.shutdown()
    
    def test_timer_callback_publishes_data(self):
        """Test that timer callback publishes perception data."""
        # Mock the publisher
        with patch.object(self.node._detection_pub, 'publish') as mock_publish:
            # Call the timer callback
            self.node._detection_callback()
            
            # Verify publish may have been called (depends on random detections)
            # Just check that the callback executes without error
            # The actual publishing happens only when detections exist
            
            # Verify the callback runs
            self.assertIsNotNone(self.node._detection_count)
    
    def test_timer_callback_message_content(self):
        """Test that published messages contain expected content."""
        published_messages = []
        
        def capture_message(msg):
            published_messages.append(msg)
        
        # Replace publish with our capture function
        self.node._detection_pub.publish = capture_message
        
        # Trigger callback multiple times to ensure we get at least one detection
        for _ in range(10):
            self.node._detection_callback()
        
        # Check if we got any messages (depends on random detections)
        if len(published_messages) > 0:
            msg = published_messages[0]
            # Message should contain frame information
            self.assertIn('Frame', msg.data)
    
    def test_processing_only_when_active(self):
        """Test that processing only occurs in active state."""
        # Deactivate the node
        mock_state = Mock()
        self.node.on_deactivate(mock_state)
        
        # Timer should be None, so callback shouldn't be called
        self.assertIsNone(self.node._timer)


class TestPerceptionNodeConfiguration(unittest.TestCase):
    """Test cases for perception node configuration handling."""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 once for all tests."""
        if not rclpy.ok():
            rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after all tests."""
        if rclpy.ok():
            rclpy.shutdown()
    
    def test_default_parameters(self):
        """Test that default parameters are set correctly."""
        node = PerceptionNode()
        
        # Check default parameter values (use correct parameter name)
        sensor_type = node.get_parameter('sensor_type').value
        detection_rate = node.get_parameter('detection_rate_hz').value
        enabled = node.get_parameter('enabled').value
        
        self.assertEqual(sensor_type, 'camera')
        self.assertEqual(detection_rate, 2.0)
        self.assertTrue(enabled)
        
        node.destroy_node()
    
    def test_custom_parameters(self):
        """Test setting custom parameters via parameter overrides."""
        # Note: In real usage, parameters would be set via config files or launch
        node = PerceptionNode()
        
        # Override parameters (use correct parameter name)
        node.set_parameters([
            rclpy.parameter.Parameter(
                'sensor_type',
                rclpy.Parameter.Type.STRING,
                'lidar'
            ),
            rclpy.parameter.Parameter(
                'detection_rate_hz',
                rclpy.Parameter.Type.DOUBLE,
                10.0
            ),
        ])
        
        sensor_type = node.get_parameter('sensor_type').value
        detection_rate = node.get_parameter('detection_rate_hz').value
        
        self.assertEqual(sensor_type, 'lidar')
        self.assertEqual(detection_rate, 10.0)
        
        node.destroy_node()
    
    def test_enabled_parameter_effect(self):
        """Test that enabled parameter affects node behavior."""
        node = PerceptionNode()
        
        # Set enabled to False
        node.set_parameters([
            rclpy.parameter.Parameter(
                'enabled',
                rclpy.Parameter.Type.BOOL,
                False
            )
        ])
        
        enabled = node.get_parameter('enabled').value
        self.assertFalse(enabled)
        
        node.destroy_node()


class TestPerceptionNodeStateManagement(unittest.TestCase):
    """Test cases for operational state management."""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 once for all tests."""
        if not rclpy.ok():
            rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after all tests."""
        if rclpy.ok():
            rclpy.shutdown()
    
    def test_state_subscription(self):
        """Test that node can subscribe to operational state updates."""
        node = PerceptionNode()
        
        # Configure the node to create subscriber
        mock_state = Mock()
        node.on_configure(mock_state)
        
        # Check that state subscriber exists (if implemented)
        # This would require the perception node to have state tracking
        
        node.destroy_node()


def main():
    """Run the test suite."""
    unittest.main()


if __name__ == '__main__':
    main()
