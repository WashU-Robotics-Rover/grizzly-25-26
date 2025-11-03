"""
Test suite for SystemManager state machine functionality.

This module tests:
- State transitions and validation
- State change service calls
- State publishing
- Invalid transition handling
- State history tracking
"""

import unittest
import rclpy
from rclpy.lifecycle import TransitionCallbackReturn
from grizzly_stack.core.system_manager import SystemManager
from grizzly_interfaces.msg import OperationalState
from grizzly_interfaces.srv import ChangeState
import time


class TestSystemManagerStateMachine(unittest.TestCase):
    """Test cases for SystemManager operational state machine."""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 once for all tests."""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 after all tests."""
        rclpy.shutdown()
    
    def setUp(self):
        """Create a fresh SystemManager node before each test."""
        self.node = SystemManager()
        
        # Configure the node to create publishers/services
        result = self.node.on_configure(None)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        
        # Activate the node to start timers
        result = self.node.on_activate(None)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
    
    def tearDown(self):
        """Clean up the node after each test."""
        self.node.destroy_node()
    
    def test_initial_state_is_startup(self):
        """Test that the node starts in STARTUP state."""
        self.assertEqual(self.node._current_state, OperationalState.STARTUP)
    
    def test_valid_transition_startup_to_standby(self):
        """Test valid transition from STARTUP to STANDBY."""
        self.assertTrue(self.node._is_valid_transition(OperationalState.STANDBY))
    
    def test_valid_transition_standby_to_autonomous(self):
        """Test valid transition from STANDBY to AUTONOMOUS."""
        # First move to STANDBY
        self.node._current_state = OperationalState.STANDBY
        self.assertTrue(self.node._is_valid_transition(OperationalState.AUTONOMOUS))
    
    def test_invalid_transition_startup_to_autonomous(self):
        """Test that direct transition from STARTUP to AUTONOMOUS is invalid."""
        self.assertFalse(self.node._is_valid_transition(OperationalState.AUTONOMOUS))
    
    def test_invalid_transition_shutdown_to_any(self):
        """Test that SHUTDOWN is a terminal state with no valid transitions."""
        self.node._current_state = OperationalState.SHUTDOWN
        self.assertFalse(self.node._is_valid_transition(OperationalState.STANDBY))
        self.assertFalse(self.node._is_valid_transition(OperationalState.AUTONOMOUS))
    
    def test_emergency_only_to_standby(self):
        """Test that EMERGENCY state can only transition to STANDBY."""
        self.node._current_state = OperationalState.EMERGENCY
        self.assertTrue(self.node._is_valid_transition(OperationalState.STANDBY))
        self.assertFalse(self.node._is_valid_transition(OperationalState.AUTONOMOUS))
        self.assertFalse(self.node._is_valid_transition(OperationalState.MANUAL))
    
    def test_same_state_transition_allowed(self):
        """Test that transitioning to the same state is always valid (no-op)."""
        self.assertTrue(self.node._is_valid_transition(OperationalState.STARTUP))
    
    def test_state_change_service_valid_request(self):
        """Test state change service with a valid transition."""
        request = ChangeState.Request()
        request.requested_state = OperationalState.STANDBY
        request.reason = "Test transition"
        
        response = ChangeState.Response()
        response = self.node._handle_state_change_request(request, response)
        
        self.assertTrue(response.success)
        self.assertEqual(response.current_state, OperationalState.STANDBY)
        self.assertIn("STARTUP", response.message)
        self.assertIn("STANDBY", response.message)
    
    def test_state_change_service_invalid_request(self):
        """Test state change service with an invalid transition."""
        request = ChangeState.Request()
        request.requested_state = OperationalState.AUTONOMOUS
        request.reason = "Invalid test transition"
        
        response = ChangeState.Response()
        response = self.node._handle_state_change_request(request, response)
        
        self.assertFalse(response.success)
        self.assertEqual(response.current_state, OperationalState.STARTUP)
        self.assertIn("Invalid", response.message)
    
    def test_state_history_tracking(self):
        """Test that state transitions are recorded in history."""
        initial_history_length = len(self.node._state_history)
        
        # Perform a valid transition
        request = ChangeState.Request()
        request.requested_state = OperationalState.STANDBY
        request.reason = "History test"
        
        response = ChangeState.Response()
        self.node._handle_state_change_request(request, response)
        
        # Check history was updated
        self.assertEqual(len(self.node._state_history), initial_history_length + 1)
        
        # Check last history entry
        last_entry = self.node._state_history[-1]
        self.assertEqual(last_entry['from'], OperationalState.STARTUP)
        self.assertEqual(last_entry['to'], OperationalState.STANDBY)
        self.assertEqual(last_entry['reason'], "History test")
        self.assertIsNotNone(last_entry['timestamp'])
    
    def test_state_transition_callback_called(self):
        """Test that _on_state_transition is called during state changes."""
        # Override the callback to track if it was called
        callback_called = {'called': False, 'old': None, 'new': None}
        
        original_callback = self.node._on_state_transition
        def tracking_callback(old_state, new_state):
            callback_called['called'] = True
            callback_called['old'] = old_state
            callback_called['new'] = new_state
            original_callback(old_state, new_state)
        
        self.node._on_state_transition = tracking_callback
        
        # Perform a transition
        request = ChangeState.Request()
        request.requested_state = OperationalState.STANDBY
        response = ChangeState.Response()
        self.node._handle_state_change_request(request, response)
        
        # Verify callback was called
        self.assertTrue(callback_called['called'])
        self.assertEqual(callback_called['old'], OperationalState.STARTUP)
        self.assertEqual(callback_called['new'], OperationalState.STANDBY)
    
    def test_state_name_helper(self):
        """Test the _state_name helper function."""
        self.assertEqual(self.node._state_name(OperationalState.STARTUP), 'STARTUP')
        self.assertEqual(self.node._state_name(OperationalState.STANDBY), 'STANDBY')
        self.assertEqual(self.node._state_name(OperationalState.AUTONOMOUS), 'AUTONOMOUS')
        self.assertEqual(self.node._state_name(OperationalState.MANUAL), 'MANUAL')
        self.assertEqual(self.node._state_name(OperationalState.EMERGENCY), 'EMERGENCY')
        self.assertEqual(self.node._state_name(OperationalState.ERROR), 'ERROR')
        self.assertEqual(self.node._state_name(OperationalState.SHUTDOWN), 'SHUTDOWN')
        self.assertIn('UNKNOWN', self.node._state_name(99))
    
    def test_complete_state_flow(self):
        """Test a complete operational flow through multiple states."""
        # STARTUP -> STANDBY
        request = ChangeState.Request()
        request.requested_state = OperationalState.STANDBY
        response = ChangeState.Response()
        self.node._handle_state_change_request(request, response)
        self.assertTrue(response.success)
        self.assertEqual(self.node._current_state, OperationalState.STANDBY)
        
        # STANDBY -> AUTONOMOUS
        request.requested_state = OperationalState.AUTONOMOUS
        response = ChangeState.Response()
        self.node._handle_state_change_request(request, response)
        self.assertTrue(response.success)
        self.assertEqual(self.node._current_state, OperationalState.AUTONOMOUS)
        
        # AUTONOMOUS -> MANUAL
        request.requested_state = OperationalState.MANUAL
        response = ChangeState.Response()
        self.node._handle_state_change_request(request, response)
        self.assertTrue(response.success)
        self.assertEqual(self.node._current_state, OperationalState.MANUAL)
        
        # MANUAL -> STANDBY
        request.requested_state = OperationalState.STANDBY
        response = ChangeState.Response()
        self.node._handle_state_change_request(request, response)
        self.assertTrue(response.success)
        self.assertEqual(self.node._current_state, OperationalState.STANDBY)
        
        # STANDBY -> SHUTDOWN
        request.requested_state = OperationalState.SHUTDOWN
        response = ChangeState.Response()
        self.node._handle_state_change_request(request, response)
        self.assertTrue(response.success)
        self.assertEqual(self.node._current_state, OperationalState.SHUTDOWN)
        
        # Verify we have 5 transitions in history
        self.assertEqual(len(self.node._state_history), 5)
    
    def test_emergency_override_from_any_state(self):
        """Test that EMERGENCY can be triggered from most states."""
        # Move to AUTONOMOUS
        self.node._current_state = OperationalState.STANDBY
        request = ChangeState.Request()
        request.requested_state = OperationalState.AUTONOMOUS
        response = ChangeState.Response()
        self.node._handle_state_change_request(request, response)
        
        # Trigger emergency from AUTONOMOUS
        request.requested_state = OperationalState.EMERGENCY
        response = ChangeState.Response()
        self.node._handle_state_change_request(request, response)
        self.assertTrue(response.success)
        self.assertEqual(self.node._current_state, OperationalState.EMERGENCY)
    
    def test_state_publisher_exists(self):
        """Test that state publisher is created during configuration."""
        self.assertIsNotNone(self.node._state_pub)
    
    def test_state_service_exists(self):
        """Test that state change service is created during configuration."""
        self.assertIsNotNone(self.node._state_service)
    
    def test_autonomous_manual_bidirectional(self):
        """Test that AUTONOMOUS and MANUAL can transition to each other."""
        # Setup: Move to STANDBY then AUTONOMOUS
        self.node._current_state = OperationalState.STANDBY
        request = ChangeState.Request()
        request.requested_state = OperationalState.AUTONOMOUS
        response = ChangeState.Response()
        self.node._handle_state_change_request(request, response)
        
        # AUTONOMOUS -> MANUAL (direct transition)
        request.requested_state = OperationalState.MANUAL
        response = ChangeState.Response()
        self.node._handle_state_change_request(request, response)
        self.assertTrue(response.success)
        self.assertEqual(self.node._current_state, OperationalState.MANUAL)
        
        # MANUAL -> AUTONOMOUS (direct transition)
        request.requested_state = OperationalState.AUTONOMOUS
        response = ChangeState.Response()
        self.node._handle_state_change_request(request, response)
        self.assertTrue(response.success)
        self.assertEqual(self.node._current_state, OperationalState.AUTONOMOUS)


class TestSystemManagerLifecycle(unittest.TestCase):
    """Test lifecycle behavior of SystemManager."""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 once for all tests."""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 after all tests."""
        rclpy.shutdown()
    
    def setUp(self):
        """Create a fresh SystemManager node before each test."""
        self.node = SystemManager()
    
    def tearDown(self):
        """Clean up the node after each test."""
        self.node.destroy_node()
    
    def test_on_configure_success(self):
        """Test that on_configure returns SUCCESS."""
        result = self.node.on_configure(None)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
    
    def test_on_activate_success(self):
        """Test that on_activate returns SUCCESS."""
        self.node.on_configure(None)
        result = self.node.on_activate(None)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
    
    def test_on_deactivate_success(self):
        """Test that on_deactivate returns SUCCESS."""
        self.node.on_configure(None)
        self.node.on_activate(None)
        result = self.node.on_deactivate(None)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
    
    def test_on_shutdown_success(self):
        """Test that on_shutdown returns SUCCESS."""
        self.node.on_configure(None)
        self.node.on_activate(None)
        result = self.node.on_shutdown(None)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)


if __name__ == '__main__':
    unittest.main()
