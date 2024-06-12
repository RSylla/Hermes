from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define the name of the package
    package_name = 'robot_localization_pkg'
    
    # Get the package directory
    pkg_dir = get_package_share_directory(package_name)
    
    # Construct the path to the EKF configuration file
    params_file_path = os.path.join(pkg_dir, 'param', 'ekf.yaml')  # Ensure the file name is correct

    # Define the EKF node configuration with direct parameter overrides
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            params_file_path,  # Load parameters from the external YAML file
            {  # Override specific parameters directly
                'map_frame': 'map',  # Adjust these values as necessary
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',
                # Add any other parameters you wish to override here
            }
        ],
    )

    # Return a LaunchDescription object that launches the EKF node with overrides
    return LaunchDescription([
        ekf_node
    ]) 
    