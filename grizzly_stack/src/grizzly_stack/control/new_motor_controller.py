#!/usr/bin/env python3
import math
from typing import Optional

from grizzly_stack.control.motor_controller import MotorController
from grizzly_stack.core import GrizzlyLifecycleNode

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class MotorNode(GrizzlyLifecycleNode):
    """
    Differential drive motor node built on GrizzlyLifecycleNode.

    Lifecycle behavior:
    - on_configure_hook:
        * Read motor parameters
        * Create publishers/subscribers
    - on_activate_hook:
        * Reset state, send a zero command
        * Timer (from base class) starts automatically if rate_hz > 0
    - tick():
        * Runs at rate_hz when Active
        * Computes left/right wheel speeds from latest cmd_vel
        * Enforces command timeout and max speed
    - on_deactivate_hook:
        * Sends a final zero command to stop the robot
    - on_cleanup_hook:
        * Destroys pubs/subs
    """

    def __init__(self):
        # rate_hz comes from the base parameter; default is 1.0
        # You can change it by setting parameter "rate_hz" when launching.
        super().__init__('motor_node')

        # Motor parameters (declared here, loaded in on_configure_hook)
        self.declare_parameter('wheel_radius', 0.05)       # m
        self.declare_parameter('wheel_base', 0.30)         # m (distance between wheels)
        self.declare_parameter('max_motor_speed', 10.0)    # rad/s (or driver units)
        self.declare_parameter('cmd_timeout', 0.5)         # s, stop if no cmd_vel in this time

        # Internal state
        self._wheel_radius: float = 0.05
        self._wheel_base: float = 0.30
        self._max_motor_speed: float = 10.0
        self._cmd_timeout: float = 0.5

        self._target_v: float = 0.0          # linear velocity [m/s]
        self._target_w: float = 0.0          # angular velocity [rad/s]
        self._last_cmd_time_sec: Optional[float] = None

        self._left_pub = None
        self._right_pub = None
        self._cmd_sub = None

    # =====================================================================
    # Lifecycle hooks
    # =====================================================================

    def on_configure_hook(self) -> bool:
        """
        Configure publishers, subscribers, and parameters.
        Called once when going Unconfigured -> Inactive.
        """
        try:
            # Load parameters
            self._wheel_radius = self.get_parameter('wheel_radius').value
            self._wheel_base = self.get_parameter('wheel_base').value
            self._max_motor_speed = self.get_parameter('max_motor_speed').value
            self._cmd_timeout = self.get_parameter('cmd_timeout').value

            self.get_logger().info(
                f'Motor params: wheel_radius={self._wheel_radius:.3f} m, '
                f'wheel_base={self._wheel_base:.3f} m, '
                f'max_motor_speed={self._max_motor_speed:.3f}, '
                f'cmd_timeout={self._cmd_timeout:.3f} s'
            )

            # Publishers for left/right motor commands
            self._left_pub = self.create_publisher(Float32, 'left_motor_cmd', 10)
            self._right_pub = self.create_publisher(Float32, 'right_motor_cmd', 10)

            # Subscriber for velocity commands
            self._cmd_sub = self.create_subscription(
                Twist,
                'cmd_vel',
                self._cmd_vel_callback,
                10
            )

            self.publish_status('CONFIGURED')
            return True

        except Exception as e:
            self.get_logger().error(f'on_configure_hook failed: {e}')
            return False

    def on_activate_hook(self) -> bool:
        """
        Called when going Inactive -> Active.
        Reset state and send an initial stop command.
        """
        try:
            self._target_v = 0.0
            self._target_w = 0.0
            now = self.get_clock().now()
            self._last_cmd_time_sec = now.nanoseconds * 1e-9

            # Send a zero command to make sure motors are stopped on activation
            self._publish_motor_cmds(0.0, 0.0)

            self.publish_status('ACTIVE - Ready to accept cmd_vel')
            return True

        except Exception as e:
            self.get_logger().error(f'on_activate_hook failed: {e}')
            return False

    def on_deactivate_hook(self) -> bool:
        """
        Called when going Active -> Inactive.
        Ensure motors are stopped.
        """
        try:
            self._publish_motor_cmds(0.0, 0.0)
            self.publish_status('INACTIVE - Motors commanded to stop')
            return True

        except Exception as e:
            self.get_logger().error(f'on_deactivate_hook failed: {e}')
            return False

    def on_cleanup_hook(self) -> bool:
        """
        Called when going Inactive -> Unconfigured.
        Destroy publishers/subscribers.
        """
        try:
            if self._cmd_sub is not None:
                self.destroy_subscription(self._cmd_sub)
                self._cmd_sub = None

            if self._left_pub is not None:
                self.destroy_publisher(self._left_pub)
                self._left_pub = None

            if self._right_pub is not None:
                self.destroy_publisher(self._right_pub)
                self._right_pub = None

            self.publish_status('CLEANED_UP')
            return True

        except Exception as e:
            self.get_logger().error(f'on_cleanup_hook failed: {e}')
            return False

    # (Optional) you could also override on_shutdown_hook for emergency stop logic

    # =====================================================================
    # Runtime behavior
    # =====================================================================

    def _cmd_vel_callback(self, msg: Twist):
        """
        Store the latest command velocity. Actual motor commands go out in tick().
        """
        self._target_v = msg.linear.x
        self._target_w = msg.angular.z
        now = self.get_clock().now()
        self._last_cmd_time_sec = now.nanoseconds * 1e-9

        self.get_logger().debug(
            f'Received cmd_vel: v={self._target_v:.3f} m/s, '
            f'w={self._target_w:.3f} rad/s'
        )

    def tick(self):
        """
        Called periodically when Active, at rate_hz from the base class.

        - Applies command timeout (zero if stale)
        - Computes wheel angular speeds
        - Clips to max_motor_speed
        - Publishes left/right motor commands
        """
        if self._left_pub is None or self._right_pub is None:
            # Not properly configured
            return

        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        v = self._target_v
        w = self._target_w

        # Safety: timeout old cmd_vel
        if self._last_cmd_time_sec is None or \
           (now_sec - self._last_cmd_time_sec) > self._cmd_timeout:
            v = 0.0
            w = 0.0

        # Diff-drive kinematics:
        #   v  = linear velocity [m/s]
        #   w  = angular velocity [rad/s]
        #   L  = wheel_base [m]
        #   R  = wheel_radius [m]
        #
        #   v_l = (2*v - w*L) / (2*R)
        #   v_r = (2*v + w*L) / (2*R)
        try:
            v_l = (2.0 * v - w * self._wheel_base) / (2.0 * self._wheel_radius)
            v_r = (2.0 * v + w * self._wheel_base) / (2.0 * self._wheel_radius)
        except ZeroDivisionError:
            self.get_logger().error(
                'wheel_radius is zero! Check configuration parameters.'
            )
            return

        # Clip to max motor speed
        v_l = max(-self._max_motor_speed, min(self._max_motor_speed, v_l))
        v_r = max(-self._max_motor_speed, min(self._max_motor_speed, v_r))

        self._publish_motor_cmds(v_l, v_r)

    # =====================================================================
    # Helper: publish or send to hardware
    # =====================================================================

    def _publish_motor_cmds(self, left_speed: float, right_speed: float):
        """
        Publish the motor speeds.

        Replace/extend this method to send commands to an Arduino, CAN bus,
        or your actual motor driver.
        """
        if self._left_pub is None or self._right_pub is None:
            return

        left_msg = Float32()
        right_msg = Float32()
        left_msg.data = float(left_speed)
        right_msg.data = float(right_speed)

        self._left_pub.publish(left_msg)
        self._right_pub.publish(right_msg)

        self.get_logger().debug(
            f'Motor cmds: left={left_speed:.3f}, right={right_speed:.3f}'
        )
        # e.g. here you could also do: self._send_to_arduino(left_speed, right_speed)