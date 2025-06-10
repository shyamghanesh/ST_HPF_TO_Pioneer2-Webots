from setuptools import setup
import os
from glob import glob

package_name = 'st_hp_to'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models'), glob('models/*/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shyam',
    maintainer_email='shyam@example.com',
    description='ST-HPF-TO project simulation in Gazebo with ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'worker_controller = st_hp_to.worker_controller:main',
            'robot_controller = st_hp_to.robot_controller:main',
            'robot_spawner = st_hp_to.spawn_robots:main',
        ],
    },
)
