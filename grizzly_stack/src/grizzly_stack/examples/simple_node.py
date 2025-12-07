"""
Example Node Template

Shows how easy it is to create a new node.

=== TO ADD A NEW NODE ===

Step 1: Create your node file (copy this template)
Step 2: Add to setup.py:
        'my_node = grizzly_stack.mymodule.my_node:main',
Step 3: Add to config/layers.yaml:
        my_node:
          enabled: true
          layer: perception  # or planning, control
          params:
            rate_hz: 1.0
Step 4: Rebuild:
        ./grizzly.py build

That's it! The launch file auto-discovers nodes from layers.yaml.
"""

from std_msgs.msg import String
from grizzly_stack.core import GrizzlyLifecycleNode, run_node


class SimpleNode(GrizzlyLifecycleNode):
    """A minimal node template - copy and modify this."""
    
    def __init__(self):
        super().__init__('simple_node')
        self._pub = None
    
    def on_configure_hook(self) -> bool:
        """Create publishers here."""
        self._pub = self.create_publisher(String, '/simple_node/output', 10)
        return True
    
    def on_cleanup_hook(self) -> bool:
        """Destroy publishers here."""
        if self._pub:
            self.destroy_publisher(self._pub)
            self._pub = None
        return True
    
    def tick(self):
        """Main loop - called at rate_hz when active."""
        msg = String()
        # Explicitly convert to string and ensure proper encoding for CycloneDDS
        msg.data = str(f'tick #{self.tick_count}')
        self._pub.publish(msg)


def main():
    run_node(SimpleNode)


if __name__ == '__main__':
    main()
