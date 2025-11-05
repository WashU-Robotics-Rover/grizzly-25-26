"""
Test suite for SystemManager state machine functionality.

This module tests:
- State transitions and validation
- State change service calls
- State publishing
- Invalid transition handling
- State history tracking
- Layer manager integration
- Layer-based node management
"""

import unittest
from unittest.mock import Mock, MagicMock, patch, call
import rclpy
from rclpy.lifecycle import TransitionCallbackReturn
from grizzly_stack.core.system_manager import SystemManager
from grizzly_stack.core.layer_manager import LayerManager
from grizzly_interfaces.msg import OperationalState
from grizzly_interfaces.srv import ChangeState
import time
import yaml
import os
from ament_index_python.packages import get_package_share_directory


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
    
    def test_state_transition_delegates_to_layer_manager(self):
        """Test that state transitions delegate to layer manager."""
        # Mock the layer manager's handle_state_transition method
        original_handle = self.node._layer_manager.handle_state_transition
        mock_handle = Mock()
        self.node._layer_manager.handle_state_transition = mock_handle
        
        # Perform a transition
        request = ChangeState.Request()
        request.requested_state = OperationalState.AUTONOMOUS
        request.reason = "Layer manager test"
        
        # First set state to STANDBY to make AUTONOMOUS valid
        self.node._current_state = OperationalState.STANDBY
        
        response = ChangeState.Response()
        self.node._handle_state_change_request(request, response)
        
        # Verify layer manager was called
        mock_handle.assert_called_once_with(
            OperationalState.STANDBY,
            OperationalState.AUTONOMOUS
        )
        
        # Restore original method
        self.node._layer_manager.handle_state_transition = original_handle
    
    def test_layer_manager_initialized(self):
        """Test that layer manager is initialized during node creation."""
        self.assertIsNotNone(self.node._layer_manager)
        self.assertIsInstance(self.node._layer_manager, LayerManager)
    
    def test_layer_manager_node_states_initialized(self):
        """Test that node states are initialized from layer configuration."""
        # After configuration, layer manager should have nodes from config
        all_layers = self.node._layer_manager.get_all_layers()
        self.assertGreater(len(all_layers), 0)
        
        # Check that node states were initialized
        for layer_name in all_layers:
            layer_nodes = self.node._layer_manager.get_layer_nodes(layer_name)
            for node_name in layer_nodes:
                # Node states should be initialized to 'inactive' during config
                self.assertIn(node_name, self.node._node_states)
    
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
        
        # Verify layer manager is initialized
        self.assertIsNotNone(self.node._layer_manager)
    
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
    
    def test_layer_manager_created_on_init(self):
        """Test that layer manager is created during node initialization."""
        # Layer manager is created in __init__, not on_configure
        self.assertIsNotNone(self.node._layer_manager)
        self.assertIsInstance(self.node._layer_manager, LayerManager)


class TestSystemManagerLayerIntegration(unittest.TestCase):
    """Test SystemManager integration with LayerManager."""
    
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
        self.node.on_configure(None)
        self.node.on_activate(None)
    
    def tearDown(self):
        """Clean up the node after each test."""
        self.node.destroy_node()
    
    def test_layer_manager_handles_autonomous_transition(self):
        """Test that layer manager handles AUTONOMOUS state transition."""
        # Mock layer manager to track calls
        original_handle = self.node._layer_manager.handle_state_transition
        call_history = []
        
        def track_calls(old_state, new_state):
            call_history.append((old_state, new_state))
            original_handle(old_state, new_state)
        
        self.node._layer_manager.handle_state_transition = track_calls
        
        # Transition to STANDBY then AUTONOMOUS
        request = ChangeState.Request()
        request.requested_state = OperationalState.STANDBY
        response = ChangeState.Response()
        self.node._handle_state_change_request(request, response)
        
        request.requested_state = OperationalState.AUTONOMOUS
        response = ChangeState.Response()
        self.node._handle_state_change_request(request, response)
        
        # Verify layer manager was called for AUTONOMOUS transition
        self.assertIn((OperationalState.STANDBY, OperationalState.AUTONOMOUS), call_history)
    
    def test_layer_manager_handles_emergency_transition(self):
        """Test that layer manager handles EMERGENCY state transition."""
        # Mock layer manager to track calls
        original_handle = self.node._layer_manager.handle_state_transition
        call_history = []
        
        def track_calls(old_state, new_state):
            call_history.append((old_state, new_state))
            original_handle(old_state, new_state)
        
        self.node._layer_manager.handle_state_transition = track_calls
        
        # Transition to AUTONOMOUS then EMERGENCY
        self.node._current_state = OperationalState.STANDBY
        request = ChangeState.Request()
        request.requested_state = OperationalState.AUTONOMOUS
        response = ChangeState.Response()
        self.node._handle_state_change_request(request, response)
        
        request.requested_state = OperationalState.EMERGENCY
        response = ChangeState.Response()
        self.node._handle_state_change_request(request, response)
        
        # Verify layer manager was called for EMERGENCY transition
        self.assertIn((OperationalState.AUTONOMOUS, OperationalState.EMERGENCY), call_history)
    
    def test_layer_manager_handles_shutdown_transition(self):
        """Test that layer manager handles SHUTDOWN state transition."""
        # Mock layer manager to track calls
        original_handle = self.node._layer_manager.handle_state_transition
        call_history = []
        
        def track_calls(old_state, new_state):
            call_history.append((old_state, new_state))
            original_handle(old_state, new_state)
        
        self.node._layer_manager.handle_state_transition = track_calls
        
        # Transition to STANDBY then SHUTDOWN
        request = ChangeState.Request()
        request.requested_state = OperationalState.STANDBY
        response = ChangeState.Response()
        self.node._handle_state_change_request(request, response)
        
        request.requested_state = OperationalState.SHUTDOWN
        response = ChangeState.Response()
        self.node._handle_state_change_request(request, response)
        
        # Verify layer manager was called for SHUTDOWN transition
        self.assertIn((OperationalState.STANDBY, OperationalState.SHUTDOWN), call_history)


class TestLayerManagerConfigLoading(unittest.TestCase):
    """Test layer manager configuration loading from YAML."""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 once for all tests."""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 after all tests."""
        rclpy.shutdown()
    
    def setUp(self):
        """Create a mock node for layer manager."""
        self.mock_node = Mock()
        self.mock_node.get_logger.return_value = Mock()
        self.mock_node.get_logger.return_value.info = Mock()
        self.mock_node.get_logger.return_value.debug = Mock()
        self.mock_node.get_logger.return_value.warn = Mock()
        self.mock_node.get_logger.return_value.error = Mock()
    
    def test_layer_manager_loads_config(self):
        """Test that layer manager loads configuration from YAML."""
        manager = LayerManager(self.mock_node)
        
        # Should have loaded layers from config
        all_layers = manager.get_all_layers()
        self.assertGreater(len(all_layers), 0)
        
        # Should have at least the standard layers
        expected_layers = ['perception', 'planning', 'control']
        for layer in expected_layers:
            self.assertIn(layer, all_layers)
    
    def test_layer_manager_has_nodes_in_layers(self):
        """Test that layers contain expected nodes."""
        manager = LayerManager(self.mock_node)
        
        # Check perception layer
        perception_nodes = manager.get_layer_nodes('perception')
        self.assertIn('perception_node', perception_nodes)
        
        # Check planning layer
        planning_nodes = manager.get_layer_nodes('planning')
        self.assertIn('planner_node', planning_nodes)
        
        # Check control layer
        control_nodes = manager.get_layer_nodes('control')
        self.assertIn('control_node', control_nodes)
    
    def test_layer_manager_fallback_on_config_error(self):
        """Test that layer manager falls back to defaults on config error."""
        # Mock the config file to not exist
        with patch('grizzly_stack.core.layer_manager.get_package_share_directory') as mock_pkg:
            mock_pkg.side_effect = Exception("Package not found")
            
            manager = LayerManager(self.mock_node)
            
            # Should still have default layers
            all_layers = manager.get_all_layers()
            self.assertGreater(len(all_layers), 0)
    
    @patch('grizzly_stack.core.layer_manager.get_package_share_directory')
    @patch('builtins.open')
    def test_layer_manager_loads_custom_config(self, mock_open, mock_pkg):
        """Test that layer manager loads custom configuration."""
        # Setup mock config
        mock_pkg.return_value = '/mock/package/share'
        
        custom_config = {
            'layers': {
                'perception': {
                    'nodes': ['perception_node', 'custom_node'],
                    'startup_order': 1
                },
                'planning': {
                    'nodes': ['planner_node'],
                    'startup_order': 2
                }
            }
        }
        
        mock_file = MagicMock()
        mock_open.return_value.__enter__.return_value = mock_file
        mock_open.return_value.__exit__ = Mock()
        
        # Mock yaml.safe_load to return our custom config
        with patch('grizzly_stack.core.layer_manager.yaml.safe_load') as mock_yaml:
            mock_yaml.return_value = custom_config
            
            manager = LayerManager(self.mock_node)
            
            # Verify custom config was loaded
            perception_nodes = manager.get_layer_nodes('perception')
            self.assertIn('perception_node', perception_nodes)
            self.assertIn('custom_node', perception_nodes)


if __name__ == '__main__':
    unittest.main()
