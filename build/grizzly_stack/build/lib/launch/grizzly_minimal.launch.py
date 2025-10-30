from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='grizzly_stack', executable='system_manager', output='screen',
             parameters=['share/grizzly_stack/config/core.yaml'])
    ])
