import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
 

def generate_launch_description():

    # Get the directories
    bringup_dir = get_package_share_directory('nav2_bringup')
    gps_wpf_dir = get_package_share_directory('hermes_nav')
    hermes_urdf_dir = get_package_share_directory('hermes_urdf')
    params_dir = os.path.join(gps_wpf_dir, 'params')
    nav2_params = os.path.join(params_dir, 'nav2_no_map_params.yaml')
    controllers_params = os.path.join(params_dir, 'controllers.yaml')
    print('Loading controllers from:', controllers_params)

    # Path to your robot's URDF file (not a xacro file)
    urdf_file_path = os.path.join(hermes_urdf_dir, 'urdf', 'hermes_model.urdf')

    # Load the URDF file as text (since it's not a xacro file)
    with open(urdf_file_path, 'r') as infp:
        robot_description_content = infp.read()

    # RViz Launch Argument
    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='False', description='Whether to start RVIZ'
    )

    # Rewritten YAML for Navigation2 parameters
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key='', param_rewrites='', convert_types=True
    )

    # Launch robot localization (EKF and NavSat)
    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gps_wpf_dir, 'launch', 'dual_ekf_navsat.launch.py')
        )
    )

    # Launch Navigation2 for the real robot
    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',  # Use real time
            'params_file': configured_params,
            'autostart': 'True',
        }.items(),
    )

    # RViz Command
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'rviz_launch.py')
        ),
        condition=IfCondition(use_rviz)
    )

    # Robot State Publisher Node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_content}
        ],
        output='screen'
    )

    # Joint State Publisher Node (optional)
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen',
    )

    # Controller Manager Node with increased delay (30 seconds)
    controller_manager_cmd = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description_content}, controllers_params],
        output='screen',
    )

    # Diff Drive controller spawner with further delay (after controller manager starts)
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diffbot_base_controller'],
    )


    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_cmd,
            on_start=[diff_drive_controller_spawner],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_cmd,
            on_start=[joint_broad_spawner],
        )
    )
    steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steering_controller'],
        output='screen',
    )
    delayed_steering_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_cmd,
            on_start=[steering_controller_spawner],
        )
    )

    # Launch Description

    
    
    
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
    #diff_drive_spawner = Node(
    #    package="controller_manager",
    #    executable="spawner",
    #    arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"]
    #)
    static_transform_base_link_to_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_gps',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps']
    )

    
    
    # Create the launch description and populate

    ld = LaunchDescription()

    # Add actions to Launch Description
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_joint_state_publisher)
    ld.add_action(robot_localization_cmd)
    ld.add_action(navigation2_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(controller_manager_cmd)

    ld.add_action(delayed_diff_drive_spawner)
    ld.add_action(delayed_joint_broad_spawner)
    ld.add_action(delayed_steering_controller_spawner)

    #ld.add_action(rsp_cmd)
    ld.add_action(static_transform_base_link_to_gps)
    ld.add_action(static_transform_map_to_odom)
    ld.add_action(static_transform_odom_to_base_footprint)
    #ld.add_action(diff_drive_spawner)    

    return ld

