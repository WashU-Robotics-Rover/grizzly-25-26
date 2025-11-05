"""
Test Planner Node for Grizzly Robotics Stack

This module implements a lifecycle-managed planner node that serves as a working
template for real-world path planning implementations. It simulates path planning
and navigation goal processing with proper lifecycle state management.

This node can be enabled/disabled via configuration and responds to lifecycle transitions
from the System Manager for coordinated state-based activation.
"""

import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn, State
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
import random
import math


class PlannerNode(LifecycleNode):
    """
    Test Planner Node - A lifecycle-managed path planning template.
    
    Lifecycle States:
    - Unconfigured: Initial state, no resources allocated
    - Inactive: Configured but not processing (resources allocated but idle)
    - Active: Fully operational, generating paths and processing navigation goals
    - Finalized: Shutdown complete
    
    When active, this node:
    - Simulates path planning at configured rate
    - Generates navigation paths with waypoints
    - Publishes path plans and planning status
    - Handles replanning requests
    - Can be dynamically activated/deactivated by System Manager
    """
    
    def __init__(self):
        """
        Initialize the PlannerNode in the Unconfigured state.
        
        At this stage, we only declare parameters and initialize member variables.
        No active resources (publishers, timers) are created yet.
        """
        super().__init__('planner_node')
        
        # Declare parameters with defaults
        self.declare_parameter('enabled', True)  # Whether planner should run
        self.declare_parameter('planning_rate_hz', 1.0)  # Planning frequency
        self.declare_parameter('max_path_length', 50.0)  # Max path distance in meters
        self.declare_parameter('waypoint_spacing', 2.0)  # Distance between waypoints
        self.declare_parameter('planning_algorithm', 'A*')  # Algorithm: A*, RRT, etc.
        self.declare_parameter('replan_threshold', 5.0)  # Distance threshold for replanning
        
        # Initialize resources to None (created in lifecycle callbacks)
        self._timer = None
        self._path_pub = None
        self._status_pub = None
        self._goal_pub = None
        
        # State tracking
        self._plan_count = 0
        self._current_goal = None
        self._current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self._path_success_rate = 0.0
        
        self.get_logger().info('Planner node created (unconfigured state)')
    
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
        self.get_logger().info('Configuring Planner Node...')
        
        try:
            # Get parameters
            self.enabled = self.get_parameter('enabled').value
            self.planning_rate = self.get_parameter('planning_rate_hz').value
            self.max_path_length = self.get_parameter('max_path_length').value
            self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
            self.algorithm = self.get_parameter('planning_algorithm').value
            self.replan_threshold = self.get_parameter('replan_threshold').value
            
            # Check if node should be enabled
            if not self.enabled:
                self.get_logger().warn('Planner node is disabled in configuration')
                # Still configure successfully, just won't activate
            
            # Create publishers (but don't start publishing yet)
            self._path_pub = self.create_publisher(
                Path,
                '/planner/path',
                10
            )
            
            self._status_pub = self.create_publisher(
                String,
                '/planner/status',
                10
            )
            
            self._goal_pub = self.create_publisher(
                PoseStamped,
                '/planner/current_goal',
                10
            )
            
            self.get_logger().info(
                f'Planner configured: algorithm={self.algorithm}, '
                f'rate={self.planning_rate}Hz, enabled={self.enabled}'
            )
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to configure planner node: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: State):
        """
        Lifecycle callback: Inactive -> Active transition.
        
        Start active path planning. This is called when the node should
        begin its operational duties.
        
        Args:
            state: The current lifecycle state
            
        Returns:
            TransitionCallbackReturn.SUCCESS if activation succeeded
            TransitionCallbackReturn.FAILURE if activation failed
        """
        self.get_logger().info('Activating Planner Node...')
        
        try:
            # Check if enabled
            if not self.enabled:
                self.get_logger().warn('Cannot activate: planner node is disabled')
                return TransitionCallbackReturn.FAILURE
            
            # Start the planning timer
            timer_period = 1.0 / self.planning_rate  # Convert Hz to seconds
            self._timer = self.create_timer(timer_period, self._planning_callback)
            
            # Reset counters and state
            self._plan_count = 0
            self._path_success_rate = 0.0
            self._current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            
            # Generate initial goal
            self._generate_new_goal()
            
            # Publish activation status
            self._publish_status('ACTIVE - Path planning started')
            
            self.get_logger().info(
                f'Planner node activated: Running {self.algorithm} planning '
                f'at {self.planning_rate}Hz'
            )
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to activate planner node: {e}')
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
        self.get_logger().info('Deactivating Planner Node...')
        
        try:
            # Stop the planning timer
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
            
            # Publish deactivation status
            self._publish_status(
                f'INACTIVE - Completed {self._plan_count} plans, '
                f'success rate: {self._path_success_rate:.1f}%'
            )
            
            self.get_logger().info(
                f'Planner node deactivated after {self._plan_count} planning cycles'
            )
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to deactivate planner node: {e}')
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
        self.get_logger().info('Cleaning up Planner Node...')
        
        try:
            # Destroy publishers
            if self._path_pub is not None:
                self.destroy_publisher(self._path_pub)
                self._path_pub = None
            
            if self._status_pub is not None:
                self.destroy_publisher(self._status_pub)
                self._status_pub = None
                
            if self._goal_pub is not None:
                self.destroy_publisher(self._goal_pub)
                self._goal_pub = None
            
            # Clear timer (should already be None from deactivate)
            self._timer = None
            
            # Clear current goal
            self._current_goal = None
            
            self.get_logger().info('Planner node cleaned up')
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to cleanup planner node: {e}')
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
        self.get_logger().info('Shutting down Planner Node...')
        
        try:
            # Stop timer if running
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
            
            # Destroy publishers if they exist
            if self._path_pub is not None:
                self.destroy_publisher(self._path_pub)
                self._path_pub = None
            
            if self._status_pub is not None:
                self.destroy_publisher(self._status_pub)
                self._status_pub = None
                
            if self._goal_pub is not None:
                self.destroy_publisher(self._goal_pub)
                self._goal_pub = None
            
            self.get_logger().info('Planner node shutdown complete')
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to shutdown planner node: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def _planning_callback(self):
        """
        Timer callback that simulates path planning processing.
        
        This method is called periodically when the node is Active. It simulates
        generating a path from current position to the goal.
        """
        if self._current_goal is None:
            self._generate_new_goal()
            return
        
        # Calculate distance to goal
        distance_to_goal = self._calculate_distance(
            self._current_position,
            self._current_goal
        )
        
        # Check if we need to replan or generate new goal
        if distance_to_goal < self.replan_threshold:
            self.get_logger().info('Goal reached! Generating new goal...')
            self._generate_new_goal()
            return
        
        # Generate path to goal
        path = self._generate_path(self._current_goal)
        
        # Simulate path success/failure
        planning_successful = random.random() > 0.1  # 90% success rate
        
        if planning_successful and path:
            self._publish_path(path)
            
            # Simulate moving along the path
            self._simulate_movement(path)
            
            # Update success rate
            self._path_success_rate = (
                (self._path_success_rate * self._plan_count + 100.0) / 
                (self._plan_count + 1)
            )
        else:
            self.get_logger().warn('Path planning failed, retrying...')
            self._path_success_rate = (
                (self._path_success_rate * self._plan_count) / 
                (self._plan_count + 1)
            )
        
        # Update counter
        self._plan_count += 1
        
        # Periodically publish status
        if self._plan_count % 5 == 0:
            self._publish_status(
                f'ACTIVE - Completed {self._plan_count} plans, '
                f'distance to goal: {distance_to_goal:.2f}m, '
                f'success rate: {self._path_success_rate:.1f}%'
            )
    
    def _generate_new_goal(self):
        """Generate a new random navigation goal."""
        angle = random.uniform(0, 2 * math.pi)
        distance = random.uniform(10.0, self.max_path_length)
        
        self._current_goal = {
            'x': self._current_position['x'] + distance * math.cos(angle),
            'y': self._current_position['y'] + distance * math.sin(angle),
            'z': random.uniform(-0.5, 0.5),
            'yaw': random.uniform(-math.pi, math.pi)
        }
        
        self.get_logger().info(
            f'New goal generated: ({self._current_goal["x"]:.2f}, '
            f'{self._current_goal["y"]:.2f}, {self._current_goal["z"]:.2f})'
        )
        
        # Publish the new goal
        self._publish_goal()
    
    def _generate_path(self, goal):
        """
        Generate a mock path from current position to goal.
        
        Args:
            goal: Dictionary with x, y, z coordinates of the goal
            
        Returns:
            List of waypoint dictionaries with x, y, z, yaw
        """
        path = []
        
        # Calculate total distance
        dx = goal['x'] - self._current_position['x']
        dy = goal['y'] - self._current_position['y']
        dz = goal['z'] - self._current_position['z']
        total_distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # Calculate number of waypoints
        num_waypoints = max(2, int(total_distance / self.waypoint_spacing))
        
        # Generate waypoints with some randomness to simulate realistic paths
        for i in range(num_waypoints):
            t = i / (num_waypoints - 1)  # Interpolation parameter (0 to 1)
            
            # Base interpolation
            x = self._current_position['x'] + t * dx
            y = self._current_position['y'] + t * dy
            z = self._current_position['z'] + t * dz
            
            # Add slight deviation to simulate obstacle avoidance
            deviation = math.sin(t * math.pi) * 0.5
            perpendicular_angle = math.atan2(dy, dx) + math.pi / 2
            x += deviation * math.cos(perpendicular_angle)
            y += deviation * math.sin(perpendicular_angle)
            
            # Calculate yaw (heading towards next point)
            if i < num_waypoints - 1:
                yaw = math.atan2(dy, dx)
            else:
                yaw = goal['yaw']
            
            waypoint = {
                'x': x,
                'y': y,
                'z': z,
                'yaw': yaw
            }
            path.append(waypoint)
        
        return path
    
    def _simulate_movement(self, path):
        """
        Simulate the robot moving along the path.
        
        Args:
            path: List of waypoints
        """
        if len(path) >= 2:
            # Move towards the second waypoint (simulate progress)
            next_waypoint = path[1]
            step_size = 0.3  # meters per planning cycle
            
            dx = next_waypoint['x'] - self._current_position['x']
            dy = next_waypoint['y'] - self._current_position['y']
            dz = next_waypoint['z'] - self._current_position['z']
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            if distance > 0:
                # Normalize and scale by step size
                self._current_position['x'] += (dx / distance) * step_size
                self._current_position['y'] += (dy / distance) * step_size
                self._current_position['z'] += (dz / distance) * step_size
    
    def _calculate_distance(self, pos1, pos2):
        """
        Calculate Euclidean distance between two positions.
        
        Args:
            pos1: Dictionary with x, y, z keys
            pos2: Dictionary with x, y, z keys
            
        Returns:
            Distance in meters
        """
        dx = pos2['x'] - pos1['x']
        dy = pos2['y'] - pos1['y']
        dz = pos2['z'] - pos1['z']
        return math.sqrt(dx**2 + dy**2 + dz**2)
    
    def _publish_path(self, waypoints):
        """
        Publish the planned path.
        
        Args:
            waypoints: List of waypoint dictionaries
        """
        if self._path_pub is None:
            return
        
        # Create Path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        # Add waypoints to path
        for wp in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            
            pose_stamped.pose.position.x = wp['x']
            pose_stamped.pose.position.y = wp['y']
            pose_stamped.pose.position.z = wp['z']
            
            # Convert yaw to quaternion (simplified - only rotation around z-axis)
            yaw = wp['yaw']
            pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
            pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)
            
            path_msg.poses.append(pose_stamped)
        
        self._path_pub.publish(path_msg)
        
        self.get_logger().debug(
            f'Published path with {len(waypoints)} waypoints, '
            f'total length: {len(waypoints) * self.waypoint_spacing:.2f}m'
        )
    
    def _publish_goal(self):
        """Publish the current navigation goal."""
        if self._goal_pub is None or self._current_goal is None:
            return
        
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        
        goal_msg.pose.position.x = self._current_goal['x']
        goal_msg.pose.position.y = self._current_goal['y']
        goal_msg.pose.position.z = self._current_goal['z']
        
        # Convert yaw to quaternion
        yaw = self._current_goal['yaw']
        goal_msg.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(yaw / 2.0)
        
        self._goal_pub.publish(goal_msg)
    
    def _publish_status(self, status_text):
        """
        Publish planner status message.
        
        Args:
            status_text: Status message string
        """
        if self._status_pub is None:
            return
        
        msg = String()
        msg.data = f'[{self.get_clock().now().nanoseconds / 1e9:.2f}] {status_text}'
        self._status_pub.publish(msg)
        
        self.get_logger().info(f'Status: {status_text}')


def main(args=None):
    """
    Main entry point for the planner_node.
    
    This function:
    1. Initializes the ROS2 Python client library
    2. Creates an instance of the PlannerNode lifecycle node
    3. Spins the node to process callbacks
    4. Cleans up when the node is terminated
    
    Note: As a lifecycle node, external lifecycle management commands are needed
    to transition the node through its states (configure, activate, etc.)
    """
    rclpy.init(args=args)
    
    # Create the PlannerNode instance (starts in Unconfigured state)
    node = PlannerNode()
    
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
