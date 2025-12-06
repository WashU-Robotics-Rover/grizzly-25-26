#!/usr/bin/env python3
import math

from grizzly_stack.control.motor_controller import MotorController
from grizzly_stack.core import GrizzlyLifecycleNode

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class DifferentialDriveMotorNode(GrizzlyLifecycleNode):
    """
    Simple motor controller node for a differential drive robot.

    - Subscribes: /cmd_vel (geometry_msgs/Twist)
    - Publishes:  /left_motor_cmd (std_msgs/Float32)
                  /right_motor_cmd (std_msgs/Float32)

    Converts linear + angular velocity commands to left/right wheel speeds.
    """

    def __init__(self):
        super().__init__('differential_drive_motor_node')

        # Declare parameters with defaults
        self.declare_parameter('wheel_radius', 0.05)      # meters
        self.declare_parameter('wheel_base', 0.30)        # distance between wheels [m]
        self.declare_parameter('max_motor_speed', 10.0)   # rad/s or whatever your driver expects

        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.max_motor_speed = self.get_parameter('max_motor_speed').get_parameter_value().double_value

        # Subscribers and publishers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.left_pub = self.create_publisher(Float32, 'left_motor_cmd', 10)
        self.right_pub = self.create_publisher(Float32, 'right_motor_cmd', 10)

        self.get_logger().info('DifferentialDriveMotorNode started.')

    def cmd_vel_callback(self, msg: Twist):
        """
        Convert Twist (linear.x, angular.z) into left/right wheel speeds.
        Classic diff-drive kinematics:

        v  = linear velocity (m/s)
        w  = angular velocity (rad/s)
        L  = wheel_base (m)
        R  = wheel_radius (m)

        v_l = (2*v - w*L) / (2*R)
        v_r = (2*v + w*L) / (2*R)
        """
        v = msg.linear.x
        w = msg.angular.z

        v_l = (2.0 * v - w * self.wheel_base) / (2.0 * self.wheel_radius)
        v_r = (2.0 * v + w * self.wheel_base) / (2.0 * self.wheel_radius)

        # Clip to max motor speed
        v_l = max(-self.max_motor_speed, min(self.max_motor_speed, v_l))
        v_r = max(-self.max_motor_speed, min(self.max_motor_speed, v_r))

        # Log (optional)
        self.get_logger().debug(f"cmd_vel: v={v:.2f}, w={w:.2f} -> v_l={v_l:.2f}, v_r={v_r:.2f}")

        # Publish left/right commands
        left_msg = Float32()
        right_msg = Float32()
        left_msg.data = float(v_l)
        right_msg.data = float(v_r)

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

        # If you were talking to an Arduino over serial, here is where
        # you’d convert v_l, v_r to whatever protocol you need.


def main(args=None):
    rclpy.init(args=args)

    node = DifferentialDriveMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down motor node.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()