import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    hermes_urdf_dir = get_package_share_directory('hermes_urdf')
    urdf_file_path = os.path.join(hermes_urdf_dir, 'urdf', 'hermes_model.urdf')
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        #parameters=[{'robot_description': Command(['xacro ', urdf_file_path])}]
        #parameters={'robot_description': Command(['xacro ', urdf_file_path, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time]), 'use_sim_time': use_sim_time}
        parameters=[{'robot_description': Command(['xacro ', urdf_file_path, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time]), 'use_sim_time': use_sim_time}]

    )
    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),

        node_robot_state_publisher
    ])
