"""Setup for grizzly_stack ROS 2 package."""

from setuptools import setup

package_name = 'grizzly_stack'

setup(
    name=package_name,
    version='0.2.0',
    packages=[
        package_name,
        package_name + '.core',
        package_name + '.perception',
        package_name + '.planner',
        package_name + '.control',
        package_name + '.examples',
    ],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/grizzly_minimal.launch.py']),
        ('share/' + package_name + '/config', [
            'config/core.yaml',
            'config/layers.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='WashU Rover Team',
    author_email='you@washu.edu',
    description='Grizzly Robotics Stack',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Core (don't modify)
            'system_manager = grizzly_stack.core.system_manager:main',
            'lifecycle_manager = grizzly_stack.core.lifecycle_manager:main',
            
            # Perception
            'camera_node = grizzly_stack.perception.camera_node:main',
            
            # Planning
            'path_planner = grizzly_stack.planner.path_planner:main',
            
            # Control
            'motor_controller = grizzly_stack.control.motor_controller:main',
            
            # Examples
            'simple_node = grizzly_stack.examples.simple_node:main',
        ],
    },
)
