from setuptools import setup
import os
from glob import glob

package_name = 'gps_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}.nodes'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'pyyaml',
        'pyproj',
        'tkinter'
    ],
    zip_safe=True,
    maintainer='hermes',
    maintainer_email='hermes@example.com',
    description='GPS waypoint navigation with Stanley controller',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'gps_waypoint_logger = gps_nav.nodes.gps_waypoint_logger:main',
            'stanley_controller = gps_nav.nodes.stanley_controller:main',
        ],
    },
)
