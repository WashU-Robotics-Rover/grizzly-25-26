import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SystemManager(Node):
    def __init__(self):
        super().__init__('system_manager')
        self.declare_parameter('health_rate_hz', 1.0)
        self.pub = self.create_publisher(String, '/system/health', 10)
        self.create_timer(1.0 / float(self.get_parameter('health_rate_hz').value), self._tick)

    def _tick(self):
        self.pub.publish(String(data='OK'))

def main():
    rclpy.init()
    node = SystemManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
