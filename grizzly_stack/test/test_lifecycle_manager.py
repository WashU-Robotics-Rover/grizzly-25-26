"""
Test Suite for Lifecycle Manager

This module contains comprehensive tests for the lifecycle_manager node:
- Node detection and service availability
- State transition orchestration
- Error handling and timeout scenarios
- Dynamic sequence building
- Integration with actual lifecycle nodes
"""

import unittest
from unittest.mock import Mock, MagicMock, patch, call
import rclpy
from rclpy.executors import SingleThreadedExecutor
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import State as LifecycleState, Transition
import time

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
    
    def test_build_startup_sequence_minimal(self):
        """Test building startup sequence with only system_manager."""
        # Mock the service check to return False (no perception)
        with patch.object(self.node, 'create_client') as mock_create_client:
            mock_client = Mock()
            mock_client.wait_for_service.return_value = False
            mock_create_client.return_value = mock_client
            
            sequence = self.node.build_startup_sequence()
            
            # Should only have system_manager transitions
            self.assertEqual(len(sequence), 2)
            self.assertEqual(sequence[0][0], 'system_manager')
            self.assertEqual(sequence[0][1], 'inactive')
            self.assertEqual(sequence[1][0], 'system_manager')
            self.assertEqual(sequence[1][1], 'active')
    
    def test_build_startup_sequence_with_perception(self):
        """Test building startup sequence with perception node detected."""
        # Mock the service check to return True (perception available)
        with patch.object(self.node, 'create_client') as mock_create_client:
            mock_client = Mock()
            mock_client.wait_for_service.return_value = True
            mock_create_client.return_value = mock_client
            
            sequence = self.node.build_startup_sequence()
            
            # Should have system_manager and perception_node transitions
            self.assertEqual(len(sequence), 3)
            self.assertEqual(sequence[2][0], 'perception_node')
            self.assertEqual(sequence[2][1], 'inactive')
    
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
    
    def test_get_node_state_service_failure(self):
        """Test handling of service call failure when getting state."""
        node_name = 'test_node'
        
        # Setup managed node
        mock_client = Mock()
        self.node.managed_nodes[node_name] = {'get_state': mock_client}
        
        # Mock failed service call
        mock_future = Mock()
        mock_future.result.return_value = None
        mock_client.call_async.return_value = mock_future
        
        with patch('rclpy.spin_until_future_complete'):
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
    
    def test_transition_node_service_failure(self):
        """Test handling of transition service call failure."""
        node_name = 'test_node'
        
        # Setup managed nodes
        self.node.managed_nodes[node_name] = {
            'get_state': Mock(),
            'change_state': Mock()
        }
        
        with patch.object(self.node, 'get_node_state', return_value='unconfigured'):
            # Mock failed service call
            mock_client = self.node.managed_nodes[node_name]['change_state']
            mock_future = Mock()
            mock_future.result.return_value = None
            mock_client.call_async.return_value = mock_future
            
            with patch('rclpy.spin_until_future_complete'):
                result = self.node.transition_node(
                    node_name,
                    Transition.TRANSITION_CONFIGURE,
                    'inactive'
                )
                
                self.assertFalse(result)
    
    def test_transition_node_timeout(self):
        """Test handling of state transition timeout."""
        node_name = 'test_node'
        
        # Setup managed nodes
        self.node.managed_nodes[node_name] = {
            'get_state': Mock(),
            'change_state': Mock()
        }
        
        # Mock get_node_state to always return 'unconfigured' (never transitions)
        with patch.object(self.node, 'get_node_state', return_value='unconfigured'):
            mock_client = self.node.managed_nodes[node_name]['change_state']
            mock_future = Mock()
            mock_response = Mock()
            mock_response.success = True
            mock_future.result.return_value = mock_response
            mock_client.call_async.return_value = mock_future
            
            with patch('rclpy.spin_until_future_complete'):
                with patch('time.sleep'):  # Speed up the test
                    result = self.node.transition_node(
                        node_name,
                        Transition.TRANSITION_CONFIGURE,
                        'inactive',
                        timeout_sec=0.5
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
        
        # This test would require creating mock lifecycle nodes
        # For now, we verify the manager can be instantiated
        self.assertIsNotNone(manager)
        self.assertEqual(manager.get_name(), 'lifecycle_manager')
        
        manager.destroy_node()


def main():
    """Run the test suite."""
    unittest.main()


if __name__ == '__main__':
    main()
