"""
Minimal Launch File for Grizzly Rover System

This launch file starts the core system_manager lifecycle node and automatically
transitions it through its lifecycle states (Configure -> Activate).

Lifecycle Management:
- Uses event-driven state transitions based on actual state changes
- Configures the node when it starts (OnProcessStart event)
- Activates the node when configuration successfully completes (OnStateTransition event)

This approach is robust against slow compute and timing variations.
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
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
    
    # --- EVENT-DRIVEN LIFECYCLE TRANSITIONS ---
    
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
    
    # --- EVENT HANDLERS FOR AUTOMATIC STATE TRANSITIONS ---
    
    # Register an event handler to configure the node when it starts
    # This triggers the CONFIGURE transition immediately after the process starts
    # No fixed timing delay - responds to actual process start event
    configure_on_start = RegisterEventHandler(
        OnProcessStart(
            target_action=system_manager_node,  # Watch for this node's process to start
            on_start=[configure_event],         # When it starts, trigger configure
        )
    )
    
    # Register an event handler to activate the node when configuration completes
    # This triggers the ACTIVATE transition only after successful configuration
    # Robust against slow compute - waits for actual state change, not fixed time
    activate_on_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=system_manager_node,  # Watch this lifecycle node
            start_state='configuring',                   # When transitioning from configuring
            goal_state='inactive',                       # To inactive (configuration complete)
            entities=[activate_event],                   # Trigger the activate event
        )
    )
    
    # Return the complete launch description with all components
    # The order matters: node must be declared before the event handlers that watch it
    return LaunchDescription([
        system_manager_node,      # Start the lifecycle node
        configure_on_start,       # Configure when process starts
        activate_on_configure,    # Activate when configuration completes
    ])
