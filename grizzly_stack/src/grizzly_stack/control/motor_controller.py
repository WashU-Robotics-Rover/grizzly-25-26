"""
Motor Controller Template

A basic control node that publishes velocity commands.
Extend this for real motor control.
"""

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from grizzly_stack.core import GrizzlyLifecycleNode, run_node


class MotorController(GrizzlyLifecycleNode):
    """Controls motor velocities."""
    
    def __init__(self):
        super().__init__('motor_controller')
        
        # Parameters
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 0.5)
        
        self._cmd_pub = None
        self._status_pub = None
        self._target_linear = 0.0
        self._target_angular = 0.0
    
    def on_configure_hook(self) -> bool:
        self._max_linear = self.get_parameter('max_linear_speed').value
        self._max_angular = self.get_parameter('max_angular_speed').value
        
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._status_pub = self.create_publisher(String, '/control/status', 10)
        
        return True
    
    def on_deactivate_hook(self) -> bool:
        # Send stop command when deactivating
        self._send_stop()
        return True
    
    def on_cleanup_hook(self) -> bool:
        if self._cmd_pub:
            self.destroy_publisher(self._cmd_pub)
        if self._status_pub:
            self.destroy_publisher(self._status_pub)
        self._cmd_pub = None
        self._status_pub = None
        return True
    
    def tick(self):
        # Publish current velocity command
        cmd = Twist()
        cmd.linear.x = self._target_linear
        cmd.angular.z = self._target_angular
        self._cmd_pub.publish(cmd)
        
        # Status update every 10 ticks
        if self.tick_count % 10 == 0:
            status = String()
            # Explicitly convert to string and ensure proper encoding for CycloneDDS
            status.data = str(f'vel: linear={self._target_linear:.2f}, angular={self._target_angular:.2f}')
            self._status_pub.publish(status)
    
    def _send_stop(self):
        """Send zero velocity command."""
        if self._cmd_pub:
            cmd = Twist()  # All zeros
            self._cmd_pub.publish(cmd)
            self.get_logger().info('Sent STOP command')
    
    def set_velocity(self, linear: float, angular: float):
        """Set target velocity (clamps to max values)."""
        self._target_linear = max(-self._max_linear, min(self._max_linear, linear))
        self._target_angular = max(-self._max_angular, min(self._max_angular, angular))


def main():
    run_node(MotorController)


if __name__ == '__main__':
    main()
