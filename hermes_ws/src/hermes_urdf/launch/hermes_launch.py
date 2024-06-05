from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the 'hermes_urdf' package
    hermes_urdf_dir = get_package_share_directory('hermes_urdf')
    urdf_file_path = os.path.join(hermes_urdf_dir, 'urdf', 'hermes_model.urdf')
    sdf_file_path = os.path.join(hermes_urdf_dir, 'urdf', 'hermes_model.sdf')

    # Declare a launch argument for the GUI
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to launch the GUI or not'
    )

    # Launch Gazebo with the GazeboRosFactory factory
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '--factory', 'GazeboRosFactory', sdf_file_path],
        output='screen'
    )

    # Spawn the robot into Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'hermes', '-file', sdf_file_path],
        output='screen'
    )

    # Robot State Publisher with robot description from the file path
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_file_path])}]
    )

    # Diff Drive Controller
    diff_drive_controller = Node(
        package='diff_drive_controller',
        executable='diff_drive_controller',
        parameters=[{'robot_description': Command(['xacro ', urdf_file_path])}],
        output='screen'
    )

    # Keyboard Control
    keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        gazebo,
        spawn_robot,
        robot_state_publisher,
        diff_drive_controller,
        keyboard_node
    ])


