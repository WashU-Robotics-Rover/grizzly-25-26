"""
Core module for Grizzly Robotics Stack.

To create a new node:

    from grizzly_stack.core import GrizzlyLifecycleNode, run_node

    class MyNode(GrizzlyLifecycleNode):
        def __init__(self):
            super().__init__('my_node')
        
        def tick(self):
            pass

    def main():
        run_node(MyNode)
"""

from .base_node import GrizzlyLifecycleNode, run_node
from .layer_manager import LayerManager
from .utils import get_state_name, load_layers_config

__all__ = [
    'GrizzlyLifecycleNode',
    'run_node',
    'LayerManager',
    'get_state_name',
    'load_layers_config',
]
