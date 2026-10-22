from setuptools import setup
from glob import glob
import os

package_name = 'autonomous_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ROS2 package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Package.xml
        ('share/' + package_name, ['package.xml']),

        # Install launch files (This automatically includes octomap.launch.py if it is in the launch/ folder)
        ('share/' + package_name + '/launch', glob('launch/*.py')),

        # Install config (e.g., ekf.yaml)
        ('share/' + package_name + '/config', glob('config/*.yaml')),

        # Install RViz configs
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ajinkya',
    maintainer_email='ajinkya@todo.todo',
    description='Autonomous drone navigation, mapping, EKF, and control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Existing nodes
            'offboard_control = autonomous_drone.offboard_control:main',
            'obstacle_avoidance = autonomous_drone.obstacle_avoidance:main',
            'depth_avoidance = autonomous_drone.depth_avoidance:main',
            'mapping_node = autonomous_drone.mapping_node:main',
            'path_planner = autonomous_drone.path_planner:main',
            'odom_tf = autonomous_drone.odom_tf:main',
            'map_merger = autonomous_drone.map_merger:main',
            # --- NEW NODES ---
            # 1. The Bridge Node (for OctoMap)
            'bridge_node = autonomous_drone.bridge_node:main',
        ],
    },
)