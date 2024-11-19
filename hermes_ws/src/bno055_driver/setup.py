from setuptools import setup
import os
from glob import glob

package_name = 'bno055_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='hermes',
    maintainer_email='raivo.sylla@gmail.com',
    description='ROS2 driver for Bosch BNO055 IMU sensor',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno055_node = bno055_driver.bno055_node:main'
        ],
    },
)