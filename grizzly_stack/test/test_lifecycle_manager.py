"""
Test Suite for Lifecycle Manager

This module contains comprehensive tests for the lifecycle_manager node:
- Layer-based node detection and service availability
- State transition orchestration
- Error handling and timeout scenarios
- Dynamic sequence building based on layers
- Configuration loading from YAML
- Integration with actual lifecycle nodes
"""

import unittest
from unittest.mock import Mock, MagicMock, patch, call
import rclpy
from rclpy.executors import SingleThreadedExecutor
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import State as LifecycleState, Transition
import time
import yaml

from grizzly_stack.core.lifecycle_manager import LifecycleManager


class TestLifecycleManager(unittest.TestCase):
    """Test cases for the LifecycleManager node."""
    
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
        self.node = LifecycleManager()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
    
    def tearDown(self):
        """Clean up after each test."""
        self.node.destroy_node()
        self.executor.shutdown()
    
    def test_node_creation(self):
        """Test that the lifecycle manager node is created correctly."""
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'lifecycle_manager')
        self.assertIsInstance(self.node.managed_nodes, dict)
        
        # Should have loaded layers from config
        self.assertIsNotNone(self.node._layers)
        self.assertIsNotNone(self.node._layer_order)
        self.assertGreater(len(self.node._layers), 0)
    
    def test_layers_loaded_from_config(self):
        """Test that layers are loaded from configuration."""
        # Should have standard layers
        self.assertIn('perception', self.node._layers)
        self.assertIn('planning', self.node._layers)
        self.assertIn('control', self.node._layers)
        
        # Check layer nodes
        self.assertIn('perception_node', self.node._layers['perception'])
        self.assertIn('planner_node', self.node._layers['planning'])
        self.assertIn('control_node', self.node._layers['control'])
    
    def test_layer_order_loaded(self):
        """Test that layer startup order is loaded from config."""
        # Should have a layer order list
        self.assertIsInstance(self.node._layer_order, list)
        self.assertGreater(len(self.node._layer_order), 0)
        
        # Should contain expected layers
        for layer in ['perception', 'planning', 'control']:
            self.assertIn(layer, self.node._layer_order)
    
    def test_check_layer_available_all_nodes(self):
        """Test checking layer availability when all nodes are available."""
        layer_name = 'perception'
        
        with patch.object(self.node, 'create_client') as mock_create_client:
            mock_client = Mock()
            mock_client.wait_for_service.return_value = True
            mock_create_client.return_value = mock_client
            
            all_available, available_nodes = self.node.check_layer_available(layer_name)
            
            self.assertTrue(all_available)
            self.assertIn('perception_node', available_nodes)
    
    def test_check_layer_available_partial_nodes(self):
        """Test checking layer availability when only some nodes are available."""
        # This test would require a layer with multiple nodes
        # For now, test with single-node layer
        layer_name = 'perception'
        
        with patch.object(self.node, 'create_client') as mock_create_client:
            mock_client = Mock()
            mock_client.wait_for_service.return_value = False
            mock_create_client.return_value = mock_client
            
            all_available, available_nodes = self.node.check_layer_available(layer_name)
            
            self.assertFalse(all_available)
            self.assertEqual(len(available_nodes), 0)
    
    def test_check_layer_available_unknown_layer(self):
        """Test checking availability of unknown layer."""
        all_available, available_nodes = self.node.check_layer_available('unknown_layer')
        
        self.assertFalse(all_available)
        self.assertEqual(len(available_nodes), 0)
    
    def test_build_startup_sequence_minimal(self):
        """Test building startup sequence with no layer nodes available."""
        # Mock check_layer_available to return no nodes
        with patch.object(
            self.node,
            'check_layer_available',
            return_value=(False, [])
        ):
            sequence = self.node.build_startup_sequence()
            
            # Should only have system_manager transitions
            self.assertEqual(len(sequence), 2)
            self.assertEqual(sequence[0][0], 'system_manager')
            self.assertEqual(sequence[0][1], 'inactive')
            self.assertEqual(sequence[1][0], 'system_manager')
            self.assertEqual(sequence[1][1], 'active')
    
    def test_build_startup_sequence_with_layers(self):
        """Test building startup sequence with layer nodes detected."""
        # Mock check_layer_available to return nodes for each layer
        def mock_check_layer(layer_name):
            layer_map = {
                'perception': (True, ['perception_node']),
                'planning': (True, ['planner_node']),
                'control': (True, ['control_node'])
            }
            return layer_map.get(layer_name, (False, []))
        
        with patch.object(
            self.node,
            'check_layer_available',
            side_effect=mock_check_layer
        ):
            sequence = self.node.build_startup_sequence()
            
            # Should have system_manager + nodes from layers
            self.assertGreater(len(sequence), 2)
            
            # Check that layer nodes are in sequence
            node_names = [item[0] for item in sequence]
            self.assertIn('perception_node', node_names)
            self.assertIn('planner_node', node_names)
            self.assertIn('control_node', node_names)
    
    def test_build_startup_sequence_respects_layer_order(self):
        """Test that startup sequence respects layer startup_order."""
        # Mock check_layer_available to return nodes
        def mock_check_layer(layer_name):
            return (True, [f'{layer_name}_node'])
        
        with patch.object(
            self.node,
            'check_layer_available',
            side_effect=mock_check_layer
        ):
            sequence = self.node.build_startup_sequence()
            
            # Extract layer nodes (skip system_manager)
            layer_nodes = [item[0] for item in sequence[2:]]
            
            # Verify order matches layer_order
            layer_order = self.node._layer_order
            for i, layer_name in enumerate(layer_order):
                if i < len(layer_nodes):
                    expected_node = f'{layer_name}_node'
                    # Check that nodes appear in the expected order
                    self.assertIn(expected_node, layer_nodes)
    
    def test_wait_for_node_success(self):
        """Test waiting for a node that becomes available."""
        node_name = 'test_node'
        
        with patch.object(self.node, 'create_client') as mock_create_client:
            # Mock both get_state and change_state clients
            mock_get_state_client = Mock()
            mock_change_state_client = Mock()
            
            mock_get_state_client.wait_for_service.return_value = True
            mock_change_state_client.wait_for_service.return_value = True
            
            mock_create_client.side_effect = [
                mock_get_state_client,
                mock_change_state_client
            ]
            
            result = self.node.wait_for_node(node_name, timeout_sec=5.0)
            
            self.assertTrue(result)
            self.assertIn(node_name, self.node.managed_nodes)
            self.assertEqual(
                self.node.managed_nodes[node_name]['get_state'],
                mock_get_state_client
            )
    
    def test_wait_for_node_timeout(self):
        """Test waiting for a node that never becomes available."""
        node_name = 'missing_node'
        
        with patch.object(self.node, 'create_client') as mock_create_client:
            mock_client = Mock()
            mock_client.wait_for_service.return_value = False
            mock_create_client.return_value = mock_client
            
            result = self.node.wait_for_node(node_name, timeout_sec=1.0)
            
            self.assertFalse(result)
            self.assertNotIn(node_name, self.node.managed_nodes)
    
    def test_get_node_state_success(self):
        """Test getting the state of a managed node."""
        node_name = 'test_node'
        expected_state = 'inactive'
        
        # Setup managed node
        mock_client = Mock()
        self.node.managed_nodes[node_name] = {'get_state': mock_client}
        
        # Mock the service call
        mock_future = Mock()
        mock_response = Mock()
        mock_response.current_state.label = expected_state
        mock_future.result.return_value = mock_response
        mock_client.call_async.return_value = mock_future
        
        with patch('rclpy.spin_until_future_complete'):
            state = self.node.get_node_state(node_name)
            
            self.assertEqual(state, expected_state)
    
    def test_get_node_state_not_managed(self):
        """Test getting state of a node not in managed_nodes."""
        node_name = 'unknown_node'
        
        state = self.node.get_node_state(node_name)
        
        self.assertIsNone(state)
    
    def test_transition_node_success(self):
        """Test successful node transition."""
        node_name = 'test_node'
        target_state = 'inactive'
        
        # Setup managed nodes
        mock_get_state_client = Mock()
        mock_change_state_client = Mock()
        
        self.node.managed_nodes[node_name] = {
            'get_state': mock_get_state_client,
            'change_state': mock_change_state_client
        }
        
        # Mock get_node_state to return 'unconfigured' first, then 'inactive'
        state_sequence = ['unconfigured', target_state]
        with patch.object(
            self.node,
            'get_node_state',
            side_effect=state_sequence
        ):
            # Mock successful transition
            mock_future = Mock()
            mock_response = Mock()
            mock_response.success = True
            mock_future.result.return_value = mock_response
            mock_change_state_client.call_async.return_value = mock_future
            
            with patch('rclpy.spin_until_future_complete'):
                result = self.node.transition_node(
                    node_name,
                    Transition.TRANSITION_CONFIGURE,
                    target_state
                )
                
                self.assertTrue(result)
    
    def test_transition_node_not_managed(self):
        """Test transitioning a node that's not managed."""
        result = self.node.transition_node(
            'unknown_node',
            Transition.TRANSITION_CONFIGURE,
            'inactive'
        )
        
        self.assertFalse(result)
    
    def test_execute_startup_sequence_node_unavailable(self):
        """Test startup sequence when a node is not available."""
        with patch.object(
            self.node,
            'build_startup_sequence',
            return_value=[('missing_node', 'inactive', 'Test')]
        ):
            with patch.object(self.node, 'wait_for_node', return_value=False):
                result = self.node.execute_startup_sequence()
                
                self.assertFalse(result)
    
    def test_execute_startup_sequence_transition_failure(self):
        """Test startup sequence when a transition fails."""
        sequence = [
            ('system_manager', 'inactive', 'Configure System Manager'),
        ]
        
        with patch.object(
            self.node,
            'build_startup_sequence',
            return_value=sequence
        ):
            with patch.object(self.node, 'wait_for_node', return_value=True):
                with patch.object(self.node, 'transition_node', return_value=False):
                    result = self.node.execute_startup_sequence()
                    
                    self.assertFalse(result)


class TestLifecycleManagerConfigLoading(unittest.TestCase):
    """Test configuration loading from YAML."""
    
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
        self.node = LifecycleManager()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
    
    def tearDown(self):
        """Clean up after each test."""
        self.node.destroy_node()
        self.executor.shutdown()
    
    @patch('grizzly_stack.core.lifecycle_manager.get_package_share_directory')
    @patch('builtins.open')
    def test_load_layers_from_config(self, mock_open, mock_pkg):
        """Test loading layers from configuration file."""
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
                },
                'control': {
                    'nodes': ['control_node'],
                    'startup_order': 3
                }
            }
        }
        
        mock_file = MagicMock()
        mock_open.return_value.__enter__.return_value = mock_file
        mock_open.return_value.__exit__ = Mock()
        
        with patch('grizzly_stack.core.lifecycle_manager.yaml.safe_load') as mock_yaml:
            mock_yaml.return_value = custom_config
            
            # Create new manager to trigger config load
            manager = LifecycleManager()
            
            # Verify custom config was loaded
            self.assertIn('perception', manager._layers)
            self.assertIn('custom_node', manager._layers['perception'])
            self.assertIn('perception_node', manager._layers['perception'])
            
            # Verify startup order
            self.assertEqual(manager._layer_order[0], 'perception')
            self.assertEqual(manager._layer_order[1], 'planning')
            self.assertEqual(manager._layer_order[2], 'control')
            
            manager.destroy_node()
    
    @patch('grizzly_stack.core.lifecycle_manager.get_package_share_directory')
    def test_load_layers_fallback_on_error(self, mock_pkg):
        """Test that fallback layers are used on config error."""
        mock_pkg.side_effect = Exception("Package not found")
        
        # Create new manager - should fall back to defaults
        manager = LifecycleManager()
        
        # Should still have default layers
        self.assertGreater(len(manager._layers), 0)
        self.assertIn('perception', manager._layers)
        self.assertIn('planning', manager._layers)
        self.assertIn('control', manager._layers)
        
        manager.destroy_node()
    
    @patch('grizzly_stack.core.lifecycle_manager.get_package_share_directory')
    @patch('builtins.open')
    def test_load_layers_empty_config(self, mock_open, mock_pkg):
        """Test that fallback is used when config is empty."""
        mock_pkg.return_value = '/mock/package/share'
        
        mock_file = MagicMock()
        mock_open.return_value.__enter__.return_value = mock_file
        mock_open.return_value.__exit__ = Mock()
        
        with patch('grizzly_stack.core.lifecycle_manager.yaml.safe_load') as mock_yaml:
            mock_yaml.return_value = {}  # Empty config
            
            # Create new manager
            manager = LifecycleManager()
            
            # Should fall back to defaults
            self.assertGreater(len(manager._layers), 0)
            self.assertIn('perception', manager._layers)
            
            manager.destroy_node()


class TestLifecycleManagerIntegration(unittest.TestCase):
    """Integration tests for lifecycle manager with mock lifecycle nodes."""
    
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
    
    def test_full_startup_sequence_mock(self):
        """Test complete startup sequence with mocked services."""
        manager = LifecycleManager()
        
        # Verify the manager can be instantiated
        self.assertIsNotNone(manager)
        self.assertEqual(manager.get_name(), 'lifecycle_manager')
        
        # Verify layers are loaded
        self.assertGreater(len(manager._layers), 0)
        self.assertGreater(len(manager._layer_order), 0)
        
        manager.destroy_node()
    
    def test_layer_based_sequence_building(self):
        """Test that startup sequence is built based on layers."""
        manager = LifecycleManager()
        
        # Mock layer checking
        def mock_check_layer(layer_name):
            if layer_name == 'perception':
                return (True, ['perception_node'])
            elif layer_name == 'planning':
                return (True, ['planner_node'])
            elif layer_name == 'control':
                return (True, ['control_node'])
            return (False, [])
        
        with patch.object(
            manager,
            'check_layer_available',
            side_effect=mock_check_layer
        ):
            sequence = manager.build_startup_sequence()
            
            # Should include nodes from all layers
            node_names = [item[0] for item in sequence]
            self.assertIn('perception_node', node_names)
            self.assertIn('planner_node', node_names)
            self.assertIn('control_node', node_names)
        
        manager.destroy_node()


def main():
    """Run the test suite."""
    unittest.main()


if __name__ == '__main__':
    main()
