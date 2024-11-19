from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ros2_bno08x_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hermes',
    maintainer_email='hermes@todo.todo',
    description='ROS2 driver for BNO08X IMU',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno08x_node = ros2_bno08x_driver.bno08x_node:main',
        ],
    },
)
