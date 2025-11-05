"""
Test Control Node for Grizzly Robotics Stack

This module implements a lifecycle-managed control node that serves as a working
template for real-world control implementations. It simulates motor control commands
and actuator management with proper lifecycle state management.

This node can be enabled/disabled via configuration and responds to lifecycle transitions
from the System Manager for coordinated state-based activation.
"""

import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn, State
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import random
import math


class ControlNode(LifecycleNode):
    """
    Test Control Node - A lifecycle-managed control template.
    
    Lifecycle States:
    - Unconfigured: Initial state, no resources allocated
    - Inactive: Configured but not processing (resources allocated but idle)
    - Active: Fully operational, generating control commands
    - Finalized: Shutdown complete
    
    When active, this node:
    - Simulates motor control command generation at configured rate
    - Publishes velocity commands and actuator states
    - Reports control status
    - Monitors control loop performance
    - Can be dynamically activated/deactivated by System Manager
    """
    
    def __init__(self):
        """
        Initialize the ControlNode in the Unconfigured state.
        
        At this stage, we only declare parameters and initialize member variables.
        No active resources (publishers, timers) are created yet.
        """
        super().__init__('control_node')
        
        # Declare parameters with defaults
        self.declare_parameter('enabled', True)  # Whether control should run
        self.declare_parameter('control_rate_hz', 10.0)  # Control loop frequency
        self.declare_parameter('max_linear_velocity', 2.0)  # Max linear velocity (m/s)
        self.declare_parameter('max_angular_velocity', 1.5)  # Max angular velocity (rad/s)
        self.declare_parameter('control_mode', 'velocity')  # Control mode: velocity, position, effort
        self.declare_parameter('safety_enabled', True)  # Enable safety checks
        self.declare_parameter('max_acceleration', 1.0)  # Max acceleration (m/s^2)
        
        # Initialize resources to None (created in lifecycle callbacks)
        self._timer = None
        self._cmd_vel_pub = None
        self._status_pub = None
        self._actuator_state_pub = None
        
        # State tracking
        self._command_count = 0
        self._current_velocity = {'linear': 0.0, 'angular': 0.0}
        self._target_velocity = {'linear': 0.0, 'angular': 0.0}
        self._control_loop_time = 0.0
        self._safety_violations = 0
        
        self.get_logger().info('Control node created (unconfigured state)')
    
    def on_configure(self, state: State):
        """
        Lifecycle callback: Unconfigured -> Inactive transition.
        
        Allocate resources like publishers and load configuration, but don't
        start active processing yet.
        
        Args:
            state: The current lifecycle state
            
        Returns:
            TransitionCallbackReturn.SUCCESS if configuration succeeded
            TransitionCallbackReturn.FAILURE if configuration failed
        """
        self.get_logger().info('Configuring Control Node...')
        
        try:
            # Get parameters
            self.enabled = self.get_parameter('enabled').value
            self.control_rate = self.get_parameter('control_rate_hz').value
            self.max_linear_vel = self.get_parameter('max_linear_velocity').value
            self.max_angular_vel = self.get_parameter('max_angular_velocity').value
            self.control_mode = self.get_parameter('control_mode').value
            self.safety_enabled = self.get_parameter('safety_enabled').value
            self.max_accel = self.get_parameter('max_acceleration').value
            
            # Check if node should be enabled
            if not self.enabled:
                self.get_logger().warn('Control node is disabled in configuration')
                # Still configure successfully, just won't activate
            
            # Create publishers (but don't start publishing yet)
            self._cmd_vel_pub = self.create_publisher(
                Twist,
                '/control/cmd_vel',
                10
            )
            
            self._status_pub = self.create_publisher(
                String,
                '/control/status',
                10
            )
            
            self._actuator_state_pub = self.create_publisher(
                String,
                '/control/actuator_state',
                10
            )
            
            self.get_logger().info(
                f'Control configured: mode={self.control_mode}, '
                f'rate={self.control_rate}Hz, enabled={self.enabled}'
            )
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to configure control node: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: State):
        """
        Lifecycle callback: Inactive -> Active transition.
        
        Start active control processing. This is called when the node should
        begin its operational duties.
        
        Args:
            state: The current lifecycle state
            
        Returns:
            TransitionCallbackReturn.SUCCESS if activation succeeded
            TransitionCallbackReturn.FAILURE if activation failed
        """
        self.get_logger().info('Activating Control Node...')
        
        try:
            # Check if enabled
            if not self.enabled:
                self.get_logger().warn('Cannot activate: control node is disabled')
                return TransitionCallbackReturn.FAILURE
            
            # Start the control timer
            timer_period = 1.0 / self.control_rate  # Convert Hz to seconds
            self._timer = self.create_timer(timer_period, self._control_callback)
            
            # Reset counters and state
            self._command_count = 0
            self._current_velocity = {'linear': 0.0, 'angular': 0.0}
            self._target_velocity = {'linear': 0.0, 'angular': 0.0}
            self._safety_violations = 0
            
            # Publish activation status
            self._publish_status('ACTIVE - Control loop running')
            
            self.get_logger().info(
                f'Control node activated: Running {self.control_mode} control '
                f'at {self.control_rate}Hz'
            )
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to activate control node: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def on_deactivate(self, state: State):
        """
        Lifecycle callback: Active -> Inactive transition.
        
        Stop active processing but keep resources allocated. This allows quick
        reactivation without full reconfiguration.
        
        Args:
            state: The current lifecycle state
            
        Returns:
            TransitionCallbackReturn.SUCCESS if deactivation succeeded
            TransitionCallbackReturn.FAILURE if deactivation failed
        """
        self.get_logger().info('Deactivating Control Node...')
        
        try:
            # Stop the control timer
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
            
            # Send zero velocity command for safety
            self._send_stop_command()
            
            # Publish deactivation status
            self._publish_status(
                f'INACTIVE - Processed {self._command_count} control commands'
            )
            
            self.get_logger().info(
                f'Control node deactivated after {self._command_count} commands'
            )
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to deactivate control node: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def on_cleanup(self, state: State):
        """
        Lifecycle callback: Inactive -> Unconfigured transition.
        
        Release all allocated resources. This is the inverse of configure.
        
        Args:
            state: The current lifecycle state
            
        Returns:
            TransitionCallbackReturn.SUCCESS if cleanup succeeded
            TransitionCallbackReturn.FAILURE if cleanup failed
        """
        self.get_logger().info('Cleaning up Control Node...')
        
        try:
            # Destroy publishers
            if self._cmd_vel_pub is not None:
                self.destroy_publisher(self._cmd_vel_pub)
                self._cmd_vel_pub = None
            
            if self._status_pub is not None:
                self.destroy_publisher(self._status_pub)
                self._status_pub = None
            
            if self._actuator_state_pub is not None:
                self.destroy_publisher(self._actuator_state_pub)
                self._actuator_state_pub = None
            
            # Clear timer (should already be None from deactivate)
            self._timer = None
            
            self.get_logger().info('Control node cleaned up')
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to cleanup control node: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def on_shutdown(self, state: State):
        """
        Lifecycle callback: Any state -> Finalized transition.
        
        Emergency shutdown from any state. Clean up any resources that might exist.
        
        Args:
            state: The current lifecycle state
            
        Returns:
            TransitionCallbackReturn.SUCCESS if shutdown succeeded
            TransitionCallbackReturn.FAILURE if shutdown failed
        """
        self.get_logger().info('Shutting down Control Node...')
        
        try:
            # Stop timer if running
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
            
            # Send emergency stop command
            self._send_stop_command()
            
            # Destroy publishers if they exist
            if self._cmd_vel_pub is not None:
                self.destroy_publisher(self._cmd_vel_pub)
                self._cmd_vel_pub = None
            
            if self._status_pub is not None:
                self.destroy_publisher(self._status_pub)
                self._status_pub = None
            
            if self._actuator_state_pub is not None:
                self.destroy_publisher(self._actuator_state_pub)
                self._actuator_state_pub = None
            
            self.get_logger().info('Control node shutdown complete')
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to shutdown control node: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def _control_callback(self):
        """
        Timer callback that simulates control command generation.
        
        This method is called periodically when the node is Active. It simulates
        generating motor control commands and managing actuator states.
        """
        # Simulate target velocity (mock navigation input)
        # In a real system, this would come from a planner or teleop
        if self._command_count % 50 == 0:  # Change target every 5 seconds at 10Hz
            self._target_velocity['linear'] = random.uniform(-self.max_linear_vel * 0.8, 
                                                              self.max_linear_vel * 0.8)
            self._target_velocity['angular'] = random.uniform(-self.max_angular_vel * 0.8, 
                                                               self.max_angular_vel * 0.8)
        
        # Simulate smooth acceleration towards target (ramp control)
        dt = 1.0 / self.control_rate
        max_delta_v = self.max_accel * dt
        
        # Linear velocity control
        linear_error = self._target_velocity['linear'] - self._current_velocity['linear']
        if abs(linear_error) > max_delta_v:
            linear_error = math.copysign(max_delta_v, linear_error)
        self._current_velocity['linear'] += linear_error
        
        # Angular velocity control
        angular_error = self._target_velocity['angular'] - self._current_velocity['angular']
        if abs(angular_error) > max_delta_v:
            angular_error = math.copysign(max_delta_v, angular_error)
        self._current_velocity['angular'] += angular_error
        
        # Safety checks
        if self.safety_enabled:
            if not self._check_safety_limits():
                self._safety_violations += 1
                self.get_logger().warn(
                    f'Safety violation detected! Total violations: {self._safety_violations}'
                )
                # Clamp to safe values
                self._current_velocity['linear'] = max(-self.max_linear_vel, 
                                                       min(self.max_linear_vel, 
                                                           self._current_velocity['linear']))
                self._current_velocity['angular'] = max(-self.max_angular_vel, 
                                                        min(self.max_angular_vel, 
                                                            self._current_velocity['angular']))
        
        # Publish control commands
        self._publish_control_command()
        
        # Update counter
        self._command_count += 1
        
        # Periodically publish status and actuator state
        if self._command_count % 50 == 0:
            self._publish_status(
                f'ACTIVE - Commands: {self._command_count}, '
                f'Linear: {self._current_velocity["linear"]:.2f}m/s, '
                f'Angular: {self._current_velocity["angular"]:.2f}rad/s'
            )
            self._publish_actuator_state()
    
    def _check_safety_limits(self):
        """
        Check if current velocities are within safety limits.
        
        Returns:
            bool: True if within limits, False otherwise
        """
        if abs(self._current_velocity['linear']) > self.max_linear_vel:
            return False
        if abs(self._current_velocity['angular']) > self.max_angular_vel:
            return False
        return True
    
    def _publish_control_command(self):
        """
        Publish velocity control command.
        """
        if self._cmd_vel_pub is None:
            return
        
        # Create Twist message
        msg = Twist()
        msg.linear.x = self._current_velocity['linear']
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self._current_velocity['angular']
        
        self._cmd_vel_pub.publish(msg)
        
        self.get_logger().debug(
            f'Published command: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}'
        )
    
    def _send_stop_command(self):
        """
        Send a zero velocity command to stop all motion.
        """
        if self._cmd_vel_pub is None:
            return
        
        msg = Twist()
        # All fields default to 0.0
        self._cmd_vel_pub.publish(msg)
        
        self.get_logger().info('Sent STOP command (zero velocity)')
    
    def _publish_status(self, status_text):
        """
        Publish control status message.
        
        Args:
            status_text: Status message string
        """
        if self._status_pub is None:
            return
        
        msg = String()
        msg.data = f'[{self.get_clock().now().nanoseconds / 1e9:.2f}] {status_text}'
        self._status_pub.publish(msg)
        
        self.get_logger().info(f'Status: {status_text}')
    
    def _publish_actuator_state(self):
        """
        Publish simulated actuator state information.
        """
        if self._actuator_state_pub is None:
            return
        
        # Simulate actuator states (in a real system, read from hardware)
        actuator_states = {
            'left_motor': {
                'velocity': self._current_velocity['linear'] - self._current_velocity['angular'] * 0.5,
                'current': abs(self._current_velocity['linear']) * 2.5,  # Simulated current draw
                'temperature': 25.0 + abs(self._current_velocity['linear']) * 10.0,
                'status': 'OK'
            },
            'right_motor': {
                'velocity': self._current_velocity['linear'] + self._current_velocity['angular'] * 0.5,
                'current': abs(self._current_velocity['linear']) * 2.5,
                'temperature': 25.0 + abs(self._current_velocity['linear']) * 10.0,
                'status': 'OK'
            }
        }
        
        state_str = f"[Command {self._command_count}] Actuator States:\n"
        for actuator_name, state in actuator_states.items():
            state_str += (
                f"  {actuator_name}: "
                f"vel={state['velocity']:.2f}m/s, "
                f"current={state['current']:.2f}A, "
                f"temp={state['temperature']:.1f}°C, "
                f"status={state['status']}\n"
            )
        
        msg = String()
        msg.data = state_str
        self._actuator_state_pub.publish(msg)
        
        self.get_logger().debug('Published actuator states')


def main(args=None):
    """
    Main entry point for the control_node.
    
    This function:
    1. Initializes the ROS2 Python client library
    2. Creates an instance of the ControlNode lifecycle node
    3. Spins the node to process callbacks
    4. Cleans up when the node is terminated
    
    Note: As a lifecycle node, external lifecycle management commands are needed
    to transition the node through its states (configure, activate, etc.)
    """
    rclpy.init(args=args)
    
    # Create the ControlNode instance (starts in Unconfigured state)
    node = ControlNode()
    
    # Spin the node - processes callbacks until shutdown
    # During spinning, the node responds to lifecycle transition commands
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Clean up the node when spinning stops
    node.destroy_node()
    
    # Shutdown the ROS2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
