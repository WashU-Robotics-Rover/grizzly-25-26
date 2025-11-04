"""
Setup configuration for the grizzly_stack ROS 2 Python package.

This file defines how the grizzly_stack package is installed, including:
- Which Python packages to include
- Where to install launch files, config files, and other resources
- Which executable entry points (nodes) to create
- Package metadata (version, description, license, etc.)
"""

from setuptools import setup

# Define the package name - used throughout the configuration
package_name = 'grizzly_stack'

setup(
    name=package_name,
    version='0.1.0',
    
    # Specify all Python packages to include
    # Main package + core subpackage (more subpackages like perception, planning, etc. to be added)
    packages=[package_name, package_name + '.core', package_name + '.perception', package_name + '.planner', package_name + '.control'],
    
    # Map package names to their directory locations
    # 'src' means the src directory is the root of grizzly_stack package
    package_dir={'': 'src'},
    
    # Define data files to install in the ROS 2 install space
    # These files are installed to specific locations so ROS 2 can find them
    data_files=[
        # Register this package with the ament resource index (required for ROS 2 discovery)
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        
        # Install the package.xml manifest file (required for ROS 2 packages)
        ('share/' + package_name, ['package.xml']),
        
        # Install launch files to share/<package_name>/launch
        # Launch files define how to start multiple nodes with specific configurations
        ('share/' + package_name + '/launch', [
            'launch/grizzly_minimal.launch.py',
        ]),
        
        # Install configuration files to share/<package_name>/config
        # YAML config files contain parameter values for nodes
        ('share/' + package_name + '/config', ['config/core.yaml', 'config/perception.yaml', 'config/planner.yaml', 'config/control.yaml']),
    ],
    
    # Python dependencies required by this package
    install_requires=['setuptools'],
    
    # Whether to create a zip file (False for ROS 2 packages)
    zip_safe=True,
    
    # Package metadata
    author='WashU Rover Team',
    author_email='you@washu.edu',
    description='Core+Perception+Planning+Control+Telemetry in one package.',
    license='MIT',
    
    # Define executable entry points (ROS 2 nodes that can be run with 'ros2 run')
    # Format: 'executable_name = package.module:function'
    # These create command-line executables that can be launched individually
    entry_points={
        'console_scripts': [
            # --- CORE SUBSYSTEM ---
            # System health monitoring and coordination
            'system_manager = grizzly_stack.core.system_manager:main',
            # Lifecycle management for node orchestration
            'lifecycle_manager = grizzly_stack.core.lifecycle_manager:main',
            
            # --- PERCEPTION SUBSYSTEM ---
            # Test perception node (lifecycle-managed template)
            'perception_node = grizzly_stack.perception.perception_node:main',
            # Central sensor processing and state estimation
            'central_perception = grizzly_stack.perception.central_perception:main',
            
            # --- PLANNER SUBSYSTEM ---
            # Test planner node (lifecycle-managed template)
            'planner_node = grizzly_stack.planner.planner_node:main',
            
            # --- CONTROL SUBSYSTEM ---
            # Test control node (lifecycle-managed template)
            'control_node = grizzly_stack.control.control_node:main',
            
            # --- PLANNING SUBSYSTEM ---
            # High-level path and behavior planning for different subsystems
            'drive_planner = grizzly_stack.planning.drive_planner:main',        # Drive system planning
            'arm_planner = grizzly_stack.planning.arm_planner:main',            # Arm manipulation planning
            'science_planner = grizzly_stack.planning.science_planner:main',    # Science operations planning
            'fusion_planner = grizzly_stack.planning.fusion_planner:main',      # Multi-subsystem coordination
            
            # --- CONTROL SUBSYSTEM ---
            # Low-level controllers that execute planned actions
            'drive_controller = grizzly_stack.control.drive_controller:main',   # Drive motor control
            'arm_controller = grizzly_stack.control.arm_controller:main',       # Arm joint control
            'science_controller = grizzly_stack.control.science_controller:main', # Science tool control
            
            # --- TELEMETRY SUBSYSTEM ---
            # Data logging and monitoring
            'telemetry_unit = grizzly_stack.telemetry.telemetry_unit:main',
        ],
    },
)
