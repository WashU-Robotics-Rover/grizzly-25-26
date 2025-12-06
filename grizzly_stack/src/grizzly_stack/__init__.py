"""
Grizzly Robotics Stack

A modular, lifecycle-managed robotics framework for the WashU Rover Team.

Quick Start:
    # Create a new node by inheriting from GrizzlyLifecycleNode:
    
    from grizzly_stack.core import GrizzlyLifecycleNode, run_node
    
    class MyNode(GrizzlyLifecycleNode):
        def __init__(self):
            super().__init__('my_node')
        
        def tick(self):
            # Your main loop logic here
            pass
    
    def main():
        run_node(MyNode)

Modules:
    - core: Base classes, lifecycle management, utilities
    - perception: Sensor processing and object detection
    - planner: Path and behavior planning
    - control: Motor and actuator control
    - examples: Example nodes demonstrating patterns

Adding a New Node:
    1. Create your node class inheriting from GrizzlyLifecycleNode
    2. Add entry point to setup.py
    3. Add to layers.yaml under the appropriate layer
    4. Rebuild: colcon build --packages-select grizzly_stack
"""

__version__ = '0.2.0'
