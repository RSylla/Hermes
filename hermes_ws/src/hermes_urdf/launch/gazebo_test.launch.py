from launch import LaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the 'hermes_urdf' package
    hermes_urdf_dir = get_package_share_directory('hermes_urdf')
    urdf_file_path = os.path.join(hermes_urdf_dir, 'urdf', 'hermes_model.urdf')
    sdf_file_path = os.path.join(hermes_urdf_dir, 'sdf', 'hermes_model.sdf')

    # Declare a launch argument for the GUI
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to launch the GUI or not'
    )

    # Ignition Gazebo Server
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', sdf_file_path],
        output='screen'
    )

    # Spawn the robot into Ignition Gazebo
    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-topic', 'robot_description', '-entity', 'hermes'],
        parameters=[{'robot_description': Command(['xacro ', urdf_file_path])}],
        output='screen'
    )

    # Robot State Publisher with robot description from the file path
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_file_path])}]
    )

    # Conditionally launch the Joint State Publisher GUI or non-GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    # Diff Drive Controller
    diff_drive_controller = Node(
        package='diff_drive_controller',
        executable='diff_drive_controller',
        parameters=[{'robot_description': Command(['xacro ', urdf_file_path])}],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        ign_gazebo,
        spawn_robot,
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        diff_drive_controller
    ])

