"""
setup.py for the lss_custom_control ROS 2 package.

This file defines how the package is installed, what Python modules it contains,
and which console entry points (CLI executables) should be created by ROS 2.

It is used by:
    colcon build
    pip install .
    ros2 run lss_custom_control <entry_point_name>
"""

from setuptools import setup

package_name = 'lss_custom_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # this is correct
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotpi',
    maintainer_email='robotpi@todo.todo',
    description='Custom control scripts for LSS arm',
    license='TODO',
    tests_require=['pytest'],
        entry_points={
        'console_scripts': [
            'sequence_executor = lss_custom_control.sequence_executor:execute_sequence',
            'read_joint_position = lss_custom_control.read_joint_position:main',
            'planner_executor = lss_custom_control.planner_executor:main',
        ],
    },
)
