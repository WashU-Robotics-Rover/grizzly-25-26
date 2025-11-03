#!/usr/bin/env python3
"""
Example: Using the Perception Node with State Transitions

This example demonstrates how to:
1. Launch the system with perception
2. Monitor state transitions
3. Change operational states
4. Observe perception node activation/deactivation

Run this after launching: ros2 launch grizzly_stack grizzly_with_perception.launch.py
"""

import rclpy
from rclpy.node import Node
from grizzly_interfaces.msg import OperationalState
from grizzly_interfaces.srv import ChangeState
from std_msgs.msg import String
import time


class PerceptionStateDemo(Node):
    """
    Demo node that shows state-based perception activation.
    """
    
    def __init__(self):
        super().__init__('perception_state_demo')
        
        # Subscribe to system state
        self.state_sub = self.create_subscription(
            OperationalState,
            '/system/state',
            self.state_callback,
            10
        )
        
        # Subscribe to perception detections
        self.detection_sub = self.create_subscription(
            String,
            '/perception/detections',
            self.detection_callback,
            10
        )
        
        # Client for changing states
        self.state_client = self.create_client(
            ChangeState,
            '/system/change_state'
        )
        
        self.current_state = None
        self.detection_count = 0
        
        self.get_logger().info('Perception State Demo started')
        self.get_logger().info('Waiting for system to initialize...')
    
    def state_callback(self, msg):
        """Called when system state changes."""
        old_state = self.current_state
        self.current_state = msg.state
        
        if old_state != self.current_state:
            self.get_logger().info(
                f'System state changed: {msg.description}'
            )
    
    def detection_callback(self, msg):
        """Called when perception publishes detections."""
        self.detection_count += 1
        self.get_logger().info(
            f'Received detection #{self.detection_count}:\n{msg.data}'
        )
    
    def change_state(self, target_state, reason):
        """
        Request a state change.
        
        Args:
            target_state: Target OperationalState value
            reason: Reason for the change
        """
        if not self.state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('State change service not available')
            return False
        
        request = ChangeState.Request()
        request.requested_state = target_state
        request.reason = reason
        
        future = self.state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'State change successful: {response.message}')
                return True
            else:
                self.get_logger().warn(f'State change failed: {response.message}')
                return False
        else:
            self.get_logger().error('State change service call failed')
            return False
    
    def run_demo(self):
        """Run the demonstration sequence."""
        # Wait for initial state
        self.get_logger().info('Waiting for system to reach STARTUP state...')
        while self.current_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        time.sleep(2)
        
        # Sequence 1: Transition to STANDBY
        self.get_logger().info('\n=== STEP 1: Transitioning to STANDBY ===')
        self.change_state(OperationalState.STANDBY, 'Demo: entering standby')
        time.sleep(3)
        
        if self.detection_count > 0:
            self.get_logger().warn('⚠️  Unexpected: Received detections in STANDBY!')
        else:
            self.get_logger().info('✅ Correct: No detections in STANDBY mode')
        
        # Sequence 2: Transition to AUTONOMOUS (activates perception)
        self.get_logger().info('\n=== STEP 2: Transitioning to AUTONOMOUS ===')
        self.get_logger().info('Perception should activate and start publishing...')
        self.change_state(OperationalState.AUTONOMOUS, 'Demo: starting autonomous mode')
        
        # Wait and collect detections
        self.get_logger().info('Collecting detections for 10 seconds...')
        start_count = self.detection_count
        for i in range(10):
            rclpy.spin_once(self, timeout_sec=1.0)
        
        detections_received = self.detection_count - start_count
        if detections_received > 0:
            self.get_logger().info(
                f'✅ Correct: Received {detections_received} detections in AUTONOMOUS mode'
            )
        else:
            self.get_logger().warn('⚠️  No detections received - is perception running?')
        
        # Sequence 3: Return to STANDBY (deactivates perception)
        self.get_logger().info('\n=== STEP 3: Returning to STANDBY ===')
        self.get_logger().info('Perception should deactivate...')
        self.change_state(OperationalState.STANDBY, 'Demo: mission complete')
        
        # Wait and verify no detections
        self.get_logger().info('Verifying no detections for 5 seconds...')
        start_count = self.detection_count
        for i in range(5):
            rclpy.spin_once(self, timeout_sec=1.0)
        
        detections_after = self.detection_count - start_count
        if detections_after == 0:
            self.get_logger().info('✅ Correct: No detections after returning to STANDBY')
        else:
            self.get_logger().warn(
                f'⚠️  Unexpected: Received {detections_after} detections after deactivation'
            )
        
        # Sequence 4: Test MANUAL mode
        self.get_logger().info('\n=== STEP 4: Testing MANUAL mode ===')
        self.change_state(OperationalState.MANUAL, 'Demo: manual control')
        
        self.get_logger().info('Collecting detections for 5 seconds...')
        start_count = self.detection_count
        for i in range(5):
            rclpy.spin_once(self, timeout_sec=1.0)
        
        detections_manual = self.detection_count - start_count
        self.get_logger().info(
            f'Received {detections_manual} detections in MANUAL mode'
        )
        
        # Final: Return to STANDBY
        self.get_logger().info('\n=== DEMO COMPLETE ===')
        self.change_state(OperationalState.STANDBY, 'Demo finished')
        
        self.get_logger().info(f'\nTotal detections received: {self.detection_count}')
        self.get_logger().info('Demo completed successfully!')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    demo = PerceptionStateDemo()
    
    try:
        demo.run_demo()
    except KeyboardInterrupt:
        pass
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
