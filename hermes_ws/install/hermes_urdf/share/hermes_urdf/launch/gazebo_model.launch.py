from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the 'sam_bot_description' package
    hermes_urdf_dir = get_package_share_directory('hermes_urdf')
    urdf_file_path = os.path.join(hermes_urdf_dir, 'urdf', 'hermes_model.urdf')
    rviz_config_path = os.path.join(hermes_urdf_dir, 'urdf', 'config.rviz')
    world_path = os.path.join(hermes_urdf_dir, '/home/hermes/Hermes/hermes_ws/src/hermes_urdf/launch/my_world.world')

    # Declare launch arguments
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
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
        output='screen'
    )
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(hermes_urdf_dir, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Execute Process for launching Gazebo
    gazebo_launch = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        rviz_config_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity_node,
        robot_localization_node,
        gazebo_launch
    ])

