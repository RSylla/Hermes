from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'ros2_bno08x_driver'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Launch arguments
    config_file = LaunchConfiguration('config_file')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'bno08x_params.yaml'),
        description='Path to config file'
    )
    
    # Node
    bno_node = Node(
        package=pkg_name,
        executable='bno08x_node',
        name='bno08x_node',
        parameters=[config_file],
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        bno_node
    ])
