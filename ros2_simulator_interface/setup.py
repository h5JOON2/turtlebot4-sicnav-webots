import os
from glob import glob
from setuptools import setup

package_name = 'ros2_simulator_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('../simulator_world/worlds/*.wbt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SICNav Project',
    maintainer_email='user@example.com',
    description='ROS 2 Interface and Controllers for Webots Simulator with TurtleBot 4',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot4_controller = ros2_simulator_interface.controllers.turtlebot4_controller.turtlebot4_controller:main',
            'crowd_supervisor = ros2_simulator_interface.controllers.crowd_supervisor.crowd_supervisor:main',
        ],
    },
)

