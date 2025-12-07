"""
Path Planner Template

A basic planning node that generates waypoints.
Extend this for real path planning algorithms.
"""

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from grizzly_stack.core import GrizzlyLifecycleNode, run_node
import random
import math


class PathPlanner(GrizzlyLifecycleNode):
    """Generates navigation waypoints."""
    
    def __init__(self):
        super().__init__('path_planner')
        
        # Parameters
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('goal_tolerance', 0.5)
        
        self._goal_pub = None
        self._status_pub = None
        self._current_goal = None
        self._goals_completed = 0
    
    def on_configure_hook(self) -> bool:
        self._max_speed = self.get_parameter('max_speed').value
        self._tolerance = self.get_parameter('goal_tolerance').value
        
        self._goal_pub = self.create_publisher(
            PoseStamped, '/planner/goal', 10
        )
        self._status_pub = self.create_publisher(
            String, '/planner/status', 10
        )
        
        return True
    
    def on_activate_hook(self) -> bool:
        # Generate first goal on activation
        self._generate_goal()
        return True
    
    def on_cleanup_hook(self) -> bool:
        if self._goal_pub:
            self.destroy_publisher(self._goal_pub)
        if self._status_pub:
            self.destroy_publisher(self._status_pub)
        self._goal_pub = None
        self._status_pub = None
        return True
    
    def tick(self):
        # Simulate reaching goal occasionally
        if random.random() > 0.9:
            self._goals_completed += 1
            self.get_logger().info(f'Goal reached! Total: {self._goals_completed}')
            self._generate_goal()
    
    def _generate_goal(self):
        """Generate a random goal pose."""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = random.uniform(-10, 10)
        goal.pose.position.y = random.uniform(-10, 10)
        goal.pose.orientation.w = 1.0
        
        self._current_goal = goal
        self._goal_pub.publish(goal)
        
        status = String()
        # Explicitly convert to string and ensure proper encoding for CycloneDDS
        status.data = str(f'New goal: ({goal.pose.position.x:.1f}, {goal.pose.position.y:.1f})')
        self._status_pub.publish(status)


def main():
    run_node(PathPlanner)


if __name__ == '__main__':
    main()
