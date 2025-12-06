"""
Lifecycle Manager Node for Grizzly Robotics Stack

This node manages the lifecycle transitions of other nodes in the system.
It uses event-based state monitoring and orchestrates transitions in the correct order,
waiting for each transition to complete before proceeding to the next.

The lifecycle manager works with layers instead of individual nodes, preventing
it from becoming bloated as more nodes are added to the system.
"""

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition
import time

from .utils import load_layers_config


class LifecycleManager(Node):
    """
    Manages lifecycle transitions for nodes organized into layers.
    
    This node:
    1. Waits for nodes in each layer to become available
    2. Configures nodes by layer (instead of individually)
    3. Performs transitions in the correct order
    4. Waits for transitions to complete before proceeding
    
    By working with layers, the lifecycle manager doesn't need to be updated
    when new nodes are added to existing layers.
    """
    
    def __init__(self):
        super().__init__('lifecycle_manager')
        
        self.get_logger().info('Lifecycle Manager starting...')
        
        # Load layer configuration using shared utility
        self._layers, self._layer_order = load_layers_config(self.get_logger())
        
        # Track managed nodes (node_name -> clients)
        self.managed_nodes = {}
        
        # Track which nodes are in which layers
        self._layer_nodes = {}
        for layer_name, nodes in self._layers.items():
            for node_name in nodes:
                if node_name not in self._layer_nodes:
                    self._layer_nodes[node_name] = []
                self._layer_nodes[node_name].append(layer_name)
    
    def check_layer_available(self, layer_name):
        """
        Check if all nodes in a layer are available.
        
        Returns:
            tuple: (all_available, available_nodes)
        """
        if layer_name not in self._layers:
            self.get_logger().warn(f'Unknown layer: {layer_name}')
            return False, []
        
        layer_nodes = self._layers[layer_name]
        available_nodes = []
        
        for node_name in layer_nodes:
            get_state_client = self.create_client(
                GetState,
                f'/{node_name}/get_state'
            )
            
            if get_state_client.wait_for_service(timeout_sec=2.0):
                available_nodes.append(node_name)
            
            self.destroy_client(get_state_client)
        
        all_available = len(available_nodes) == len(layer_nodes)
        
        if all_available:
            self.get_logger().info(
                f'✅ {layer_name} layer available - all nodes detected: {available_nodes}'
            )
        elif available_nodes:
            self.get_logger().warn(
                f'⚠️  {layer_name} layer partially available - '
                f'nodes: {available_nodes}, missing: {set(layer_nodes) - set(available_nodes)}'
            )
        else:
            self.get_logger().info(f'ℹ️  {layer_name} layer not available - skipping')
        
        return all_available, available_nodes
    
    def build_startup_sequence(self):
        """
        Dynamically build the startup sequence based on available layers.
        
        Returns:
            list: Startup sequence as (node_name, target_state, description) tuples
        """
        sequence = [
            ('system_manager', 'inactive', 'Configure System Manager'),
            ('system_manager', 'active', 'Activate System Manager'),
        ]
        
        # Check each layer and configure all available nodes
        for layer_name in self._layer_order:
            all_available, available_nodes = self.check_layer_available(layer_name)
            
            if available_nodes:
                for node_name in available_nodes:
                    sequence.append(
                        (node_name, 'inactive', f'Configure {layer_name} layer: {node_name}')
                    )
        
        return sequence
        
    def wait_for_node(self, node_name, timeout_sec=30.0):
        """Wait for a lifecycle node's services to become available."""
        self.get_logger().info(f'Waiting for {node_name} to become available...')
        
        get_state_client = self.create_client(GetState, f'/{node_name}/get_state')
        change_state_client = self.create_client(ChangeState, f'/{node_name}/change_state')
        
        start_time = time.time()
        
        while not (get_state_client.wait_for_service(timeout_sec=1.0) and 
                   change_state_client.wait_for_service(timeout_sec=1.0)):
            if time.time() - start_time > timeout_sec:
                self.get_logger().error(f'Timeout waiting for {node_name} after {timeout_sec}s')
                return False
            
            elapsed = time.time() - start_time
            self.get_logger().info(f'Still waiting for {node_name}... ({elapsed:.1f}s elapsed)')
        
        self.managed_nodes[node_name] = {
            'get_state': get_state_client,
            'change_state': change_state_client
        }
        
        time.sleep(0.2)  # Small delay for node initialization
        
        self.get_logger().info(f'✅ {node_name} is available')
        return True
    
    def get_node_state(self, node_name, retries=3, retry_delay=0.2):
        """Get the current lifecycle state of a node with retry logic."""
        if node_name not in self.managed_nodes:
            self.get_logger().error(f'{node_name} not in managed nodes')
            return None
        
        client = self.managed_nodes[node_name]['get_state']
        
        for attempt in range(retries):
            request = GetState.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                return future.result().current_state.label
            else:
                if attempt < retries - 1:
                    self.get_logger().warn(
                        f'Failed to get state for {node_name}, retrying... ({attempt + 1}/{retries})'
                    )
                    time.sleep(retry_delay)
                else:
                    self.get_logger().error(
                        f'Failed to get state for {node_name} after {retries} attempts'
                    )
        
        return None
    
    def transition_node(self, node_name, transition_id, target_state_label, timeout_sec=10.0):
        """Transition a node to a new state and wait for completion."""
        if node_name not in self.managed_nodes:
            self.get_logger().error(f'{node_name} not in managed nodes')
            return False
        
        current_state = self.get_node_state(node_name)
        if current_state is None:
            return False
        
        self.get_logger().info(
            f'Transitioning {node_name}: {current_state} -> {target_state_label}'
        )
        
        client = self.managed_nodes[node_name]['change_state']
        request = ChangeState.Request()
        request.transition.id = transition_id
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        
        if future.result() is None or not future.result().success:
            self.get_logger().error(f'❌ Failed to transition {node_name} to {target_state_label}')
            return False
        
        # Wait for the state to actually change
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            current_state = self.get_node_state(node_name)
            if current_state == target_state_label:
                self.get_logger().info(
                    f'✅ {node_name} successfully transitioned to {target_state_label}'
                )
                return True
            time.sleep(0.1)
        
        self.get_logger().error(f'❌ Timeout waiting for {node_name} to reach {target_state_label}')
        return False
    
    def execute_startup_sequence(self):
        """Execute the complete startup sequence for all managed nodes."""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Starting Lifecycle Management Sequence')
        self.get_logger().info('=' * 60)
        
        startup_sequence = self.build_startup_sequence()
        
        # Wait for all unique nodes
        unique_nodes = set(item[0] for item in startup_sequence)
        for node_name in unique_nodes:
            if not self.wait_for_node(node_name):
                self.get_logger().error(f'Failed to detect {node_name}')
                return False
        
        self.get_logger().info('')
        self.get_logger().info('All nodes detected, starting transitions...')
        self.get_logger().info('')
        
        # Execute each transition
        for node_name, target_state, description in startup_sequence:
            self.get_logger().info(f'Step: {description}')
            
            if target_state == 'inactive':
                transition_id = Transition.TRANSITION_CONFIGURE
            elif target_state == 'active':
                transition_id = Transition.TRANSITION_ACTIVATE
            else:
                self.get_logger().error(f'Unknown target state: {target_state}')
                return False
            
            if not self.transition_node(node_name, transition_id, target_state):
                self.get_logger().error(f'Startup sequence failed at: {description}')
                return False
            
            self.get_logger().info('')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('✅ Lifecycle Management Complete - System Ready')
        self.get_logger().info('=' * 60)
        return True


def main(args=None):
    """Main entry point for the lifecycle manager."""
    rclpy.init(args=args)
    
    manager = LifecycleManager()
    
    try:
        success = manager.execute_startup_sequence()
        
        if success:
            manager.get_logger().info('Lifecycle manager will now exit (startup complete)')
        else:
            manager.get_logger().error('Lifecycle management failed')
            return 1
        
    except KeyboardInterrupt:
        manager.get_logger().info('Lifecycle manager interrupted')
    except Exception as e:
        manager.get_logger().error(f'Lifecycle manager exception: {e}')
        return 1
    finally:
        manager.destroy_node()
        rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
