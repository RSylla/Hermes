from setuptools import setup, find_packages
import os

package_name = 'robot_localization_pkg'

# Define the path for launch files and parameter files
launch_files = [os.path.join('launch', file) for file in os.listdir('launch') if file.endswith('.py')]
param_files = [os.path.join('param', 'ekf.yaml')]  # Update this if there are more parameter files

# Setup function
setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),  # Automatically find and include all packages
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), launch_files),
        # Include all parameter files
        (os.path.join('share', package_name, 'param'), param_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A ROS 2 package for robot localization.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Example: 'executable_name = package_name.module_name:function_name'
        ],
    },
)
