"""
Grizzly Rover System Launch File

This launch file starts the core system_manager lifecycle node and conditionally
includes the perception node based on configuration.

Lifecycle Management:
- Uses a dedicated lifecycle_manager node for orchestration
- Lifecycle manager waits for nodes to be ready, then transitions them
- Event-based approach adapts to system performance (no fixed timers)
- Robust and deterministic startup sequence

Conditional Node Loading:
- Reads perception.yaml to check if perception is enabled
- If enabled=true, includes and configures the perception node
- If enabled=false, skips perception node entirely
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import yaml
from ament_index_python.packages import get_package_share_directory
import os


def is_perception_enabled():
    """
    Check if perception node is enabled in the configuration file.
    
    Returns:
        bool: True if perception is enabled, False otherwise
    """
    try:
        package_share = get_package_share_directory('grizzly_stack')
        perception_config_path = os.path.join(package_share, 'config', 'perception.yaml')
        
        with open(perception_config_path, 'r') as f:
            config = yaml.safe_load(f)
            
        # Navigate the YAML structure to find the enabled parameter
        if config and 'perception_node' in config:
            params = config['perception_node'].get('ros__parameters', {})
            return params.get('enabled', False)
        
        return False
    except Exception as e:
        # If we can't read the config, assume perception is disabled
        print(f"Warning: Could not read perception config: {e}")
        return False


def generate_launch_description():
    """
    Generate the launch description for the Grizzly system.
    
    Returns:
        LaunchDescription: A launch description containing the system_manager node
                          and optionally the perception node based on configuration.
    """
    # Construct the path to the configuration files
    core_config_file = PathJoinSubstitution([
        FindPackageShare('grizzly_stack'),
        'config',
        'core.yaml'
    ])
    
    perception_config_file = PathJoinSubstitution([
        FindPackageShare('grizzly_stack'),
        'config',
        'perception.yaml'
    ])
    
    # Define the system_manager as a LifecycleNode
    # LifecycleNode requires explicit state transitions (unlike regular Node)
    system_manager_node = LifecycleNode(
        package='grizzly_stack',
        executable='system_manager',
        namespace='',
        name='system_manager',
        output='screen',
        parameters=[core_config_file],
    )
    
    # --- EVENT-DRIVEN LIFECYCLE MANAGEMENT ---
    # Use a dedicated lifecycle_manager node to orchestrate transitions
    # This is more robust than timers as it waits for actual state changes
    
    lifecycle_manager_node = Node(
        package='grizzly_stack',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
    )
    
    # Automatically shut down the lifecycle manager after it completes
    # This keeps it from running unnecessarily after startup
    shutdown_manager_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=lifecycle_manager_node,
            on_exit=lambda event, context: None,  # Just let it exit cleanly
        )
    )
    
    # Build the launch description components list
    launch_components = [
        system_manager_node,
        lifecycle_manager_node,
        shutdown_manager_on_exit,
    ]
    
    # === CONDITIONAL PERCEPTION NODE ===
    # Check if perception is enabled in configuration
    if is_perception_enabled():
        print("✅ Perception enabled in config - including perception node")
        
        # Define the perception node
        perception_node = LifecycleNode(
            package='grizzly_stack',
            executable='perception_node',
            namespace='',
            name='perception_node',
            output='screen',
            parameters=[perception_config_file],
        )
        
        # Add perception node to launch
        # The lifecycle_manager will handle configuration automatically
        launch_components.append(perception_node)
        
        # Note: Perception is configured by lifecycle_manager but NOT activated
        # System Manager will activate it based on operational state transitions
    else:
        print("ℹ️  Perception disabled in config - skipping perception node")
    
    # Return the complete launch description
    return LaunchDescription(launch_components)
