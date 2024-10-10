import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Get the directories
    bringup_dir = get_package_share_directory('nav2_bringup')
    gps_wpf_dir = get_package_share_directory("hermes_nav")
    params_dir = os.path.join(gps_wpf_dir, "params")
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    controllers_params = os.path.join(params_dir, "controllers.yaml")
    print("Loading controllers from:", controllers_params)
    
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    # RViz Launch Argument
    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='False', description='Whether to start RVIZ'
    )

    # Launch robot localization (EKF and NavSat)
    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gps_wpf_dir, 'launch', 'dual_ekf_navsat.launch.py'))
    )

    # Launch Navigation2 for the real robot
    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "False",  # Use real time
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    # RViz Command
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", 'rviz_launch.py')),
        condition=IfCondition(use_rviz)  # Correct way to conditionally launch RViz
    )
    
    controller_manager_cmd = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_params],  # Load the controllers.yaml
        output='screen'
    )
    
    rsp_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gps_wpf_dir, 'launch', 'rsp.launch.py'))  # Ensure this path is correct
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(robot_localization_cmd)
    ld.add_action(navigation2_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(controller_manager_cmd)
    ld.add_action(rsp_cmd)
    return ld

