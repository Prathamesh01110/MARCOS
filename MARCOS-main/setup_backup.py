from setuptools import setup

package_name = 'autonomous_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # This looks for a folder named "autonomous_drone"
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ajinkya',
    maintainer_email='ajinkya@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # The format is: 'node_name = package_folder.filename:main'
            'offboard_control = autonomous_drone.offboard_control:main',
            'obstacle_avoidance = autonomous_drone.obstacle_avoidance:main',
            'depth_avoidance = autonomous_drone.depth_avoidance:main',
            'mapping_node = autonomous_drone.mapping_node:main',
            
        ],
    },
)