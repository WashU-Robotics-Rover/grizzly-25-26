"""
Minimal Launch File for Grizzly Rover System

This launch file starts the core system_manager lifecycle node and automatically
transitions it through its lifecycle states (Configure -> Activate).

Lifecycle Management:
- Uses TimerAction to trigger state transitions at specific intervals
- Configures the node after 1 second (allows node to fully initialize)
- Activates the node after 2 seconds (allows configuration to complete)

This approach ensures deterministic startup and proper resource initialization.
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """
    Generate the launch description for the minimal Grizzly system.
    
    Returns:
        LaunchDescription: A launch description containing the system_manager node
                          and automatic lifecycle transition events.
    """
    # Construct the path to the configuration file using ROS 2 package discovery
    # FindPackageShare locates the installed package, then we join the path to the config file
    config_file = PathJoinSubstitution([
        FindPackageShare('grizzly_stack'),  # Find the grizzly_stack package install location
        'config',                            # Navigate to the config directory
        'core.yaml'                          # Specify the config file name
    ])
    
    # Define the system_manager as a LifecycleNode
    # LifecycleNode requires explicit state transitions (unlike regular Node)
    system_manager_node = LifecycleNode(
        package='grizzly_stack',           # The ROS 2 package containing this node
        executable='system_manager',       # The executable name (from setup.py entry_points)
        namespace='',                      # Empty namespace means node is at root level
        name='system_manager',             # The node name (must match the name in the node code)
        output='screen',                   # Print node output to the terminal
        parameters=[config_file],          # Load parameters from the YAML config file
    )
    
    # --- LIFECYCLE TRANSITION EVENTS ---
    
    # Create an event to trigger the CONFIGURE transition
    # This moves the node from Unconfigured -> Inactive state
    # During this transition, the node allocates resources (publishers, etc.)
    configure_event = EmitEvent(
        event=ChangeState(
            # Match the node by checking if 'system_manager' is in its name
            lifecycle_node_matcher=lambda node: 'system_manager' in node.name,
            # Specify which transition to trigger
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    
    # Create an event to trigger the ACTIVATE transition
    # This moves the node from Inactive -> Active state
    # During this transition, the node starts its timers and begins operation
    activate_event = EmitEvent(
        event=ChangeState(
            # Match the node by checking if 'system_manager' is in its name
            lifecycle_node_matcher=lambda node: 'system_manager' in node.name,
            # Specify which transition to trigger
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )
    
    # --- TIMER ACTIONS FOR AUTOMATIC STATE TRANSITIONS ---
    
    # Schedule the configure event to trigger 1 second after launch
    # This delay ensures the node has fully initialized before configuration
    configure_timer = TimerAction(
        period=1.0,                 # Wait 1.0 second
        actions=[configure_event],  # Then trigger the configure event
    )
    
    # Schedule the activate event to trigger 2 seconds after launch
    # This delay ensures configuration has completed before activation
    activate_timer = TimerAction(
        period=2.0,                # Wait 2.0 seconds
        actions=[activate_event],  # Then trigger the activate event
    )
    
    # Return the complete launch description with all components
    # The order matters: node must be declared before the events that affect it
    return LaunchDescription([
        system_manager_node,  # Start the lifecycle node
        configure_timer,      # Schedule automatic configuration
        activate_timer,       # Schedule automatic activation
    ])
