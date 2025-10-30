from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('grizzly_stack'),
        'config',
        'core.yaml'
    ])
    
    return LaunchDescription([
        Node(package='grizzly_stack', executable='system_manager', output='screen',
             parameters=[config_file])
    ])
