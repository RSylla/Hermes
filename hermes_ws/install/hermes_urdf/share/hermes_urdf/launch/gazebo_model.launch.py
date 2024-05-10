from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    hermes_urdf_dir = get_package_share_directory('hermes_urdf')
    urdf_file_path = os.path.join(hermes_urdf_dir, 'urdf', 'hermes_model.urdf')
    rviz_config_path = os.path.join(hermes_urdf_dir, 'urdf', 'config.rviz')

    # Launch arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=urdf_file_path,
        description='Absolute path to robot URDF file'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=rviz_config_path,
        description='Absolute path to RViz config file'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_file_path])}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_file_path]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        prefix='xterm -e',
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        rviz_config_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        teleop_node
    ])

