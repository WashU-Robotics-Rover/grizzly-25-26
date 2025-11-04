"""
Lifecycle Manager Node for Grizzly Robotics Stack

This node manages the lifecycle transitions of other nodes in the system.
It uses event-based state monitoring and orchestrates transitions in the correct order,
waiting for each transition to complete before proceeding to the next.

This is more robust than timer-based approaches as it adapts to system performance.
"""

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition, State as LifecycleState
import time


class LifecycleManager(Node):
    """
    Manages lifecycle transitions for nodes in the Grizzly system.
    
    This node:
    1. Waits for lifecycle nodes to become available
    2. Queries their current state
    3. Performs transitions in the correct order
    4. Waits for transitions to complete before proceeding
    """
    
    def __init__(self):
        super().__init__('lifecycle_manager')
        
        self.get_logger().info('Lifecycle Manager starting...')
        
        # Track managed nodes
        self.managed_nodes = {}
    
    def build_startup_sequence(self):
        """
        Dynamically build the startup sequence based on available nodes.
        
        Returns:
            list: Startup sequence as (node_name, target_state, description) tuples
        """
        sequence = [
            ('system_manager', 'inactive', 'Configure System Manager'),
            ('system_manager', 'active', 'Activate System Manager'),
        ]
        
        # Check if perception node is available
        perception_client = self.create_client(
            GetState,
            '/perception_node/get_state'
        )
        
        if perception_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Perception node detected - adding to sequence')
            sequence.append(('perception_node', 'inactive', 'Configure Perception Node'))
            # Note: perception_node stays in inactive, system_manager will activate it
        else:
            self.get_logger().info('Perception node not detected - skipping')
        
        # Clean up the test client
        self.destroy_client(perception_client)
        
        # Check if planner node is available
        planner_client = self.create_client(
            GetState,
            '/planner_node/get_state'
        )
        
        if planner_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Planner node detected - adding to sequence')
            sequence.append(('planner_node', 'inactive', 'Configure Planner Node'))
            # Note: planner_node stays in inactive, system_manager will activate it
        else:
            self.get_logger().info('Planner node not detected - skipping')
        
        # Clean up the test client
        self.destroy_client(planner_client)
        
        # Check if control node is available
        control_client = self.create_client(
            GetState,
            '/control_node/get_state'
        )
        
        if control_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Control node detected - adding to sequence')
            sequence.append(('control_node', 'inactive', 'Configure Control Node'))
            # Note: control_node stays in inactive, system_manager will activate it
        else:
            self.get_logger().info('Control node not detected - skipping')
        
        # Clean up the test client
        self.destroy_client(control_client)
        
        return sequence
        
    def wait_for_node(self, node_name, timeout_sec=30.0):
        """
        Wait for a lifecycle node's services to become available.
        
        Args:
            node_name: Name of the node to wait for
            timeout_sec: Maximum time to wait
            
        Returns:
            bool: True if node is available, False if timeout
        """
        self.get_logger().info(f'Waiting for {node_name} to become available...')
        
        get_state_client = self.create_client(
            GetState,
            f'/{node_name}/get_state'
        )
        
        change_state_client = self.create_client(
            ChangeState,
            f'/{node_name}/change_state'
        )
        
        start_time = time.time()
        
        while not (get_state_client.wait_for_service(timeout_sec=1.0) and 
                   change_state_client.wait_for_service(timeout_sec=1.0)):
            if time.time() - start_time > timeout_sec:
                self.get_logger().error(
                    f'Timeout waiting for {node_name} after {timeout_sec}s'
                )
                return False
            
            elapsed = time.time() - start_time
            self.get_logger().info(
                f'Still waiting for {node_name}... ({elapsed:.1f}s elapsed)'
            )
        
        # Store clients for later use
        self.managed_nodes[node_name] = {
            'get_state': get_state_client,
            'change_state': change_state_client
        }
        
        # Small delay to ensure node is fully initialized
        # (services can become available before node constructor completes)
        time.sleep(0.2)
        
        self.get_logger().info(f'✅ {node_name} is available')
        return True
    
    def get_node_state(self, node_name, retries=3, retry_delay=0.2):
        """
        Get the current lifecycle state of a node with retry logic.
        
        Args:
            node_name: Name of the node
            retries: Number of times to retry if the call fails
            retry_delay: Seconds to wait between retries
            
        Returns:
            str: Current state label (e.g., 'unconfigured', 'inactive', 'active')
        """
        if node_name not in self.managed_nodes:
            self.get_logger().error(f'{node_name} not in managed nodes')
            return None
        
        client = self.managed_nodes[node_name]['get_state']
        
        for attempt in range(retries):
            request = GetState.Request()
            
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                state_label = future.result().current_state.label
                return state_label
            else:
                if attempt < retries - 1:
                    self.get_logger().warn(
                        f'Failed to get state for {node_name}, retrying... ({attempt + 1}/{retries})'
                    )
                    time.sleep(retry_delay)
                else:
                    self.get_logger().error(f'Failed to get state for {node_name} after {retries} attempts')
        
        return None
    
    def transition_node(self, node_name, transition_id, target_state_label, timeout_sec=10.0):
        """
        Transition a node to a new state and wait for completion.
        
        Args:
            node_name: Name of the node
            transition_id: Transition ID to execute
            target_state_label: Expected state after transition
            timeout_sec: Maximum time to wait for transition
            
        Returns:
            bool: True if transition succeeded, False otherwise
        """
        if node_name not in self.managed_nodes:
            self.get_logger().error(f'{node_name} not in managed nodes')
            return False
        
        # Check current state
        current_state = self.get_node_state(node_name)
        if current_state is None:
            return False
        
        self.get_logger().info(
            f'Transitioning {node_name}: {current_state} -> {target_state_label}'
        )
        
        # Make the transition request
        client = self.managed_nodes[node_name]['change_state']
        request = ChangeState.Request()
        request.transition.id = transition_id
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        
        if future.result() is None or not future.result().success:
            self.get_logger().error(
                f'❌ Failed to transition {node_name} to {target_state_label}'
            )
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
        
        self.get_logger().error(
            f'❌ Timeout waiting for {node_name} to reach {target_state_label}'
        )
        return False
    
    def execute_startup_sequence(self):
        """
        Execute the complete startup sequence for all managed nodes.
        
        Returns:
            bool: True if all transitions succeeded, False otherwise
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('Starting Lifecycle Management Sequence')
        self.get_logger().info('=' * 60)
        
        # Dynamically build the startup sequence
        startup_sequence = self.build_startup_sequence()
        
        # First, wait for all unique nodes to be available
        unique_nodes = set(item[0] for item in startup_sequence)
        for node_name in unique_nodes:
            if not self.wait_for_node(node_name):
                self.get_logger().error(f'Failed to detect {node_name}')
                return False
        
        self.get_logger().info('')
        self.get_logger().info('All nodes detected, starting transitions...')
        self.get_logger().info('')
        
        # Execute each transition in sequence
        for node_name, target_state, description in startup_sequence:
            self.get_logger().info(f'Step: {description}')
            
            # Determine which transition to use based on target state
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
        # Execute the startup sequence
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
