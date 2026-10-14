import os
from glob import glob
from setuptools import setup

package_name = 'sicnav_ros2_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'sicnav_ros2_bridge'],
    package_dir={'sicnav_ros2_interface': 'sicnav_ros2_bridge'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='SICNav Project',
    maintainer_email='user@example.com',
    description='SICNav Social Robot Navigation to ROS 2 Interface',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sicnav_node = sicnav_ros2_bridge.sicnav_ros2_node:main',
        ],
    },
)

