"""
Camera Node Template

A basic perception node that simulates camera-based object detection.
Extend this for real camera/sensor processing.
"""

from std_msgs.msg import String
from grizzly_stack.core import GrizzlyLifecycleNode, run_node
import random


class CameraNode(GrizzlyLifecycleNode):
    """Simulates camera-based object detection."""
    
    def __init__(self):
        super().__init__('camera_node')
        
        # Parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('detection_threshold', 0.5)
        
        self._detection_pub = None
        self._detections = 0
    
    def on_configure_hook(self) -> bool:
        self._camera_id = self.get_parameter('camera_id').value
        self._threshold = self.get_parameter('detection_threshold').value
        
        self._detection_pub = self.create_publisher(
            String, '/perception/detections', 10
        )
        
        self.get_logger().info(f'Camera {self._camera_id} configured')
        return True
    
    def on_cleanup_hook(self) -> bool:
        if self._detection_pub:
            self.destroy_publisher(self._detection_pub)
            self._detection_pub = None
        return True
    
    def tick(self):
        # Simulate detection (replace with real camera processing)
        if random.random() > self._threshold:
            self._detections += 1
            msg = String()
            msg.data = f'Detection #{self._detections}: object at ({random.randint(0,100)}, {random.randint(0,100)})'
            self._detection_pub.publish(msg)
            
            if self._detections % 5 == 0:
                self.get_logger().info(f'Total detections: {self._detections}')


def main():
    run_node(CameraNode)


if __name__ == '__main__':
    main()
