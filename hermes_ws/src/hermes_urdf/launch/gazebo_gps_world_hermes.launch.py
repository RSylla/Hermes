import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'hermes_urdf'
    package_dir = get_package_share_directory(package_name)
    
    # Paths to the URDF file
    urdf_file = os.path.join(package_dir, 'urdf', 'hermes_model.urdf')

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

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': 'empty_world'}.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen',
        additional_env={'RCUTILS_LOGGING_BUFFERED_STREAM': '1', 'RCUTILS_CONSOLE_OUTPUT_FORMAT': '{message}'}
    )

    # Robot state publisher launch
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Static transforms
    static_transform_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    static_transform_odom_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )
    static_transform_base_link_to_liigend = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_liigend',
    arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'liigend']
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Set gazebo up to find models properly
    ld.add_action(set_gazebo_model_path_cmd)

    # Simulator launch
    ld.add_action(gazebo)

    # Robot state publisher launch
    ld.add_action(start_robot_state_publisher_cmd)

    # Spawn entity
    ld.add_action(spawn_entity)

    # Static transforms
    ld.add_action(static_transform_map_to_odom)
    ld.add_action(static_transform_odom_to_base_footprint)
    #ld.add_action(static_transform_base_link_to_liigend)

    return ld

