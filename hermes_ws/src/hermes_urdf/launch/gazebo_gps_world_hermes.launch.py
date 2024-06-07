import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    package_name = 'hermes_urdf'
    package_dir = get_package_share_directory(package_name)
    
    # Paths to the URDF and world files
    urdf_file = os.path.join(package_dir, 'urdf', 'hermes_model.urdf')
    world_file = os.path.join(package_dir, 'custom_ground_plane', 'my_world.sdf')

    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # Set the GAZEBO_MODEL_PATH environment variable
    models_dir = os.path.join(package_dir, "models")
    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path = os.environ['GAZEBO_MODEL_PATH'] + os.pathsep + models_dir
    else:
        gazebo_model_path = models_dir

    set_gazebo_model_path_cmd = SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path)

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # Static transforms
    static_transform_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    static_transform_odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set gazebo up to find models properly
    ld.add_action(set_gazebo_model_path_cmd)

    # Simulator launch
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # Robot state publisher launch
    ld.add_action(start_robot_state_publisher_cmd)

    # Spawn entity
    ld.add_action(spawn_entity_cmd)

    # Static transforms
    ld.add_action(static_transform_map_to_odom)
    ld.add_action(static_transform_odom_to_base_link)

    return ld

