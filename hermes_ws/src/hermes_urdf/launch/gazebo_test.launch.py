from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo with your robot
        Node(
            package='gazebo_ros',
            executable='gazebo',
            arguments=['-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Spawn your robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'your_robot_name'],
            output='screen'
        ),
        # Launch the teleoperation node
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            parameters=[{'joy_config': 'ps3'}], # Example configuration for a PS3 controller
            output='screen'
        )
    ])
