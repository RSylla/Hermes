from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hermes_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aleksmalm',
    maintainer_email='aleks.malm11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interactive_waypoint_follower = hermes_nav.interactive_waypoint_follower:main',
            'gps_waypoint_logger = hermes_nav.gps_waypoint_logger:main',
            'logged_waypoint_follower = hermes_nav.logged_waypoint_follower:main',
            'gps_converter_node = hermes_nav.gps_converter_node:main',
        ],
    },
)

