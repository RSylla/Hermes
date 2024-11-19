from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Declare launch configurations
    waypoints_file = LaunchConfiguration('waypoints_file')

    # Declare launch arguments
    declare_waypoints_file = DeclareLaunchArgument(
        'waypoints_file',
        default_value='/home/hermes/Hermes/hermes_ws/src/waypoints/route_latest.yaml',
        description='Path to waypoints file'
    )

    # GPS Waypoint Logger node
    gps_logger = Node(
        package='gps_nav',
        executable='gps_waypoint_logger',
        name='gps_waypoint_logger',
        output='screen'
    )

    # Stanley Controller node
    stanley_controller = Node(
        package='gps_nav',
        executable='stanley_controller',
        name='stanley_controller',
        output='screen',
        parameters=[{
            'waypoints_file': waypoints_file,
            'k_gain': 0.5,
            'target_speed': 0.5,
            'goal_threshold': 2.0,
            'look_ahead': 1.0,
            'utm_zone': 35,  # Estonia is in UTM zone 35
            'utm_band': 'N'
        }]
    )

    return LaunchDescription([
        declare_waypoints_file,
        gps_logger,
        stanley_controller
    ])
