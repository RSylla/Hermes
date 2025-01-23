from setuptools import find_packages, setup

package_name = 'waypoint_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hermes',
    maintainer_email='raivo.sylla@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_recorder_node = waypoint_follower.waypoint_recorder_node:main',
            'controller_node = waypoint_follower.controller_node:main',
        ],
    },
)
