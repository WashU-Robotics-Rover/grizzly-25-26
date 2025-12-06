"""
Core Tests for Grizzly Stack

Tests the essential functionality:
- SystemManager state transitions
- LayerManager configuration loading
- LifecycleManager startup sequence
"""

import unittest
from unittest.mock import Mock, patch
import rclpy
from rclpy.lifecycle import TransitionCallbackReturn

from grizzly_stack.core.system_manager import SystemManager
from grizzly_stack.core.layer_manager import LayerManager
from grizzly_stack.core.lifecycle_manager import LifecycleManager
from grizzly_interfaces.msg import OperationalState
from grizzly_interfaces.srv import ChangeState


class TestSystemManager(unittest.TestCase):
    """Tests for SystemManager state machine."""
    
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()
    
    def setUp(self):
        self.node = SystemManager()
        self.node.on_configure(None)
        self.node.on_activate(None)
    
    def tearDown(self):
        self.node.destroy_node()
    
    def test_initial_state_is_startup(self):
        """Node starts in STARTUP state."""
        self.assertEqual(self.node._current_state, OperationalState.STARTUP)
    
    def test_valid_transition_startup_to_standby(self):
        """Can transition from STARTUP to STANDBY."""
        self.assertTrue(self.node._is_valid_transition(OperationalState.STANDBY))
    
    def test_invalid_transition_startup_to_autonomous(self):
        """Cannot jump from STARTUP directly to AUTONOMOUS."""
        self.assertFalse(self.node._is_valid_transition(OperationalState.AUTONOMOUS))
    
    def test_state_change_service(self):
        """State change service works for valid transitions."""
        request = ChangeState.Request()
        request.requested_state = OperationalState.STANDBY
        request.reason = "Test"
        
        response = ChangeState.Response()
        response = self.node._handle_state_change_request(request, response)
        
        self.assertTrue(response.success)
        self.assertEqual(self.node._current_state, OperationalState.STANDBY)
    
    def test_complete_state_flow(self):
        """Can traverse: STARTUP -> STANDBY -> AUTONOMOUS -> MANUAL -> STANDBY."""
        states = [
            OperationalState.STANDBY,
            OperationalState.AUTONOMOUS,
            OperationalState.MANUAL,
            OperationalState.STANDBY,
        ]
        
        for target in states:
            request = ChangeState.Request()
            request.requested_state = target
            response = ChangeState.Response()
            self.node._handle_state_change_request(request, response)
            self.assertTrue(response.success)
            self.assertEqual(self.node._current_state, target)
    
    def test_emergency_from_autonomous(self):
        """Can trigger EMERGENCY from AUTONOMOUS."""
        self.node._current_state = OperationalState.AUTONOMOUS
        self.assertTrue(self.node._is_valid_transition(OperationalState.EMERGENCY))
    
    def test_shutdown_is_terminal(self):
        """SHUTDOWN state has no valid transitions."""
        self.node._current_state = OperationalState.SHUTDOWN
        self.assertFalse(self.node._is_valid_transition(OperationalState.STANDBY))


class TestLayerManager(unittest.TestCase):
    """Tests for LayerManager configuration."""
    
    def setUp(self):
        self.mock_node = Mock()
        self.mock_node.get_logger.return_value = Mock()
    
    def test_loads_layers_from_config(self):
        """LayerManager loads layer configuration."""
        manager = LayerManager(self.mock_node)
        layers = manager.get_all_layers()
        
        self.assertIn('perception', layers)
        self.assertIn('planning', layers)
        self.assertIn('control', layers)
    
    def test_fallback_on_config_error(self):
        """Falls back to defaults if config fails."""
        with patch('grizzly_stack.core.utils.get_package_share_directory') as mock:
            mock.side_effect = Exception("Not found")
            manager = LayerManager(self.mock_node)
            
            self.assertGreater(len(manager.get_all_layers()), 0)


class TestLifecycleManager(unittest.TestCase):
    """Tests for LifecycleManager."""
    
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()
    
    def test_node_creation(self):
        """LifecycleManager creates successfully."""
        node = LifecycleManager()
        
        self.assertEqual(node.get_name(), 'lifecycle_manager')
        self.assertGreater(len(node._layers), 0)
        
        node.destroy_node()
    
    def test_builds_startup_sequence(self):
        """Builds a startup sequence with system_manager."""
        node = LifecycleManager()
        
        with patch.object(node, 'check_layer_available', return_value=(False, [])):
            sequence = node.build_startup_sequence()
        
        # Should have system_manager configure and activate
        self.assertEqual(sequence[0][0], 'system_manager')
        self.assertEqual(sequence[0][1], 'inactive')
        self.assertEqual(sequence[1][0], 'system_manager')
        self.assertEqual(sequence[1][1], 'active')
        
        node.destroy_node()


if __name__ == '__main__':
    unittest.main()
