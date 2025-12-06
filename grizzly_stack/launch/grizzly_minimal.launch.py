"""
Grizzly Rover System Launch File

Automatically discovers and launches nodes from layers.yaml.
No need to modify this file when adding new nodes!
"""

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import yaml
from ament_index_python.packages import get_package_share_directory
import os


def load_node_config():
    """Load node configuration from layers.yaml."""
    try:
        package_share = get_package_share_directory('grizzly_stack')
        config_path = os.path.join(package_share, 'config', 'layers.yaml')
        
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"⚠️  Could not load layers.yaml: {e}")
        return {'nodes': {}}


def generate_launch_description():
    """Generate launch description from layers.yaml."""
    
    config = load_node_config()
    nodes_config = config.get('nodes', {})
    
    # Core config for system_manager
    core_config = PathJoinSubstitution([
        FindPackageShare('grizzly_stack'), 'config', 'core.yaml'
    ])
    
    # Always start system_manager and lifecycle_manager
    launch_components = [
        LifecycleNode(
            package='grizzly_stack',
            executable='system_manager',
            namespace='',
            name='system_manager',
            output='screen',
            parameters=[core_config],
        ),
        Node(
            package='grizzly_stack',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
        ),
    ]
    
    # Auto-discover and launch enabled nodes from config
    for node_name, node_config in nodes_config.items():
        if not node_config.get('enabled', False):
            print(f"ℹ️  {node_name} disabled")
            continue
        
        layer = node_config.get('layer', 'unknown')
        params = node_config.get('params', {})
        
        print(f"✅ Starting {node_name} ({layer} layer)")
        
        # Build parameters list
        node_params = [{'ros__parameters': params}] if params else []
        
        launch_components.append(LifecycleNode(
            package='grizzly_stack',
            executable=node_name,
            namespace='',
            name=node_name,
            output='screen',
            parameters=node_params,
        ))
    
    return LaunchDescription(launch_components)
