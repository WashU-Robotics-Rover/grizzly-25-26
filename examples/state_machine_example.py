#!/usr/bin/env python3
"""
Example script demonstrating how to interact with the System Manager state machine.

This script shows how to:
1. Subscribe to state updates
2. Request state changes
3. Handle state transitions
4. Monitor system state

Usage:
    python3 state_machine_example.py
"""

import rclpy
from rclpy.node import Node
from grizzly_interfaces.msg import OperationalState
from grizzly_interfaces.srv import ChangeState
import time


class StateMachineExample(Node):
    """Example node demonstrating state machine interaction."""
    
    def __init__(self):
        super().__init__('state_machine_example')
        
        # Create service client for state changes
        self.state_client = self.create_client(
            ChangeState, 
            '/system/change_state'
        )
        
        # Subscribe to state updates
        self.state_sub = self.create_subscription(
            OperationalState,
            '/system/state',
            self.state_callback,
            10
        )
        
        self.current_state = None
        self.get_logger().info('State Machine Example Node initialized')
    
    def state_callback(self, msg):
        """Called when state updates are published."""
        self.current_state = msg.state
        self.get_logger().info(
            f'State Update: {msg.description} (value: {msg.state})'
        )
    
    def wait_for_state_service(self, timeout_sec=5.0):
        """Wait for the state change service to become available."""
        self.get_logger().info('Waiting for state change service...')
        if not self.state_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('State change service not available!')
            return False
        self.get_logger().info('State change service is ready')
        return True
    
    def request_state_change(self, target_state, reason=""):
        """
        Request a state change and wait for the response.
        
        Args:
            target_state: Target operational state (use OperationalState constants)
            reason: Optional reason for the state change
            
        Returns:
            True if state change was successful, False otherwise
        """
        request = ChangeState.Request()
        request.requested_state = target_state
        request.reason = reason
        
        self.get_logger().info(
            f'Requesting state change to {self._state_name(target_state)}: {reason}'
        )
        
        # Call service asynchronously
        future = self.state_client.call_async(request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✓ Success: {response.message}')
                return True
            else:
                self.get_logger().warn(f'✗ Failed: {response.message}')
                return False
        else:
            self.get_logger().error('Service call failed or timed out')
            return False
    
    def wait_for_state(self, target_state, timeout_sec=10.0):
        """
        Wait until the system reaches a specific state.
        
        Args:
            target_state: State to wait for
            timeout_sec: Maximum time to wait
            
        Returns:
            True if state was reached, False if timeout
        """
        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz
        
        while time.time() - start_time < timeout_sec:
            if self.current_state == target_state:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
            rate.sleep()
        
        return False
    
    def _state_name(self, state):
        """Get human-readable name for a state."""
        state_names = {
            OperationalState.STARTUP: 'STARTUP',
            OperationalState.STANDBY: 'STANDBY',
            OperationalState.AUTONOMOUS: 'AUTONOMOUS',
            OperationalState.MANUAL: 'MANUAL',
            OperationalState.EMERGENCY: 'EMERGENCY',
            OperationalState.ERROR: 'ERROR',
            OperationalState.SHUTDOWN: 'SHUTDOWN'
        }
        return state_names.get(state, f'UNKNOWN({state})')
    
    def run_example_sequence(self):
        """Run an example sequence of state transitions."""
        self.get_logger().info('='*60)
        self.get_logger().info('Starting State Machine Example Sequence')
        self.get_logger().info('='*60)
        
        # Wait for service to be available
        if not self.wait_for_state_service():
            return False
        
        # Give some time to receive initial state
        time.sleep(1)
        rclpy.spin_once(self, timeout_sec=0.5)
        
        self.get_logger().info(f'\nInitial state: {self._state_name(self.current_state)}')
        
        # Sequence 1: STARTUP -> STANDBY
        self.get_logger().info('\n--- Step 1: Transition to STANDBY ---')
        if self.request_state_change(
            OperationalState.STANDBY,
            "Example: System ready for operations"
        ):
            self.wait_for_state(OperationalState.STANDBY)
        
        time.sleep(2)
        
        # Sequence 2: STANDBY -> AUTONOMOUS
        self.get_logger().info('\n--- Step 2: Transition to AUTONOMOUS ---')
        if self.request_state_change(
            OperationalState.AUTONOMOUS,
            "Example: Starting autonomous operations"
        ):
            self.wait_for_state(OperationalState.AUTONOMOUS)
        
        time.sleep(2)
        
        # Sequence 3: AUTONOMOUS -> MANUAL
        self.get_logger().info('\n--- Step 3: Transition to MANUAL ---')
        if self.request_state_change(
            OperationalState.MANUAL,
            "Example: Switching to manual control"
        ):
            self.wait_for_state(OperationalState.MANUAL)
        
        time.sleep(2)
        
        # Sequence 4: Test invalid transition (should fail)
        self.get_logger().info('\n--- Step 4: Try invalid transition (MANUAL -> SHUTDOWN) ---')
        self.request_state_change(
            OperationalState.SHUTDOWN,
            "Example: This should fail - must go through STANDBY"
        )
        
        time.sleep(2)
        
        # Sequence 5: MANUAL -> EMERGENCY
        self.get_logger().info('\n--- Step 5: Trigger EMERGENCY ---')
        if self.request_state_change(
            OperationalState.EMERGENCY,
            "Example: Emergency stop triggered"
        ):
            self.wait_for_state(OperationalState.EMERGENCY)
        
        time.sleep(2)
        
        # Sequence 6: EMERGENCY -> STANDBY (recovery)
        self.get_logger().info('\n--- Step 6: Recover from EMERGENCY ---')
        if self.request_state_change(
            OperationalState.STANDBY,
            "Example: Emergency cleared, returning to standby"
        ):
            self.wait_for_state(OperationalState.STANDBY)
        
        time.sleep(2)
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('Example Sequence Complete!')
        self.get_logger().info('='*60)
        
        return True


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = StateMachineExample()
    
    try:
        # Run the example sequence
        node.run_example_sequence()
        
        # Keep node alive to continue receiving state updates
        node.get_logger().info('\nPress Ctrl+C to exit...')
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
