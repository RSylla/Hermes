from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the 'hermes_urdf' package
    hermes_urdf_dir = get_package_share_directory('hermes_urdf')
    urdf_file_path = os.path.join(hermes_urdf_dir, 'urdf', 'hermes.urdf')
    rviz_config_path = os.path.join(hermes_urdf_dir, 'urdf', 'config.rviz')

    # Declare a launch argument for the GUI
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to launch the GUI or not'
    )

    # Robot State Publisher with robot description from the file path
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
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
        condition=UnlessCondition(LaunchConfiguration('gui')),
        arguments=[urdf_file_path]
    )

    # RViz Visualization
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        gui_arg,
        DeclareLaunchArgument(name='model', default_value=urdf_file_path,
                              description='Absolute path to robot URDF file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=rviz_config_path,
                              description='Absolute path to RViz configuration file'),
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz2
    ])

