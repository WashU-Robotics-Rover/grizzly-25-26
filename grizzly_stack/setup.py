from setuptools import setup

package_name = 'grizzly_stack'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.core'],
    package_dir={package_name: '.'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/grizzly_minimal.launch.py']),
        ('share/' + package_name + '/config', ['config/core.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='WashU Rover Team',
    author_email='you@washu.edu',
    description='Core+Perception+Planning+Control+Telemetry in one package.',
    license='MIT',
    entry_points={
        'console_scripts': [
            # core
            'system_manager = grizzly_stack.core.system_manager:main',
            # perception
            'central_perception = grizzly_stack.perception.central_perception:main',
            # planning
            'drive_planner = grizzly_stack.planning.drive_planner:main',
            'arm_planner = grizzly_stack.planning.arm_planner:main',
            'science_planner = grizzly_stack.planning.science_planner:main',
            'fusion_planner = grizzly_stack.planning.fusion_planner:main',
            # control
            'drive_controller = grizzly_stack.control.drive_controller:main',
            'arm_controller = grizzly_stack.control.arm_controller:main',
            'science_controller = grizzly_stack.control.science_controller:main',
            # telemetry
            'telemetry_unit = grizzly_stack.telemetry.telemetry_unit:main',
        ],
    },
)
