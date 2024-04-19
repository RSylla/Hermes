from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Declare a launch argument for the GUI
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to launch the GUI or not'
    )

    # Robot State Publisher with robot description from a file
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': "$(find-pkg-share hermes_urdf)/urdf/hermes.urdf"}]
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

    # RViz Visualization
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', '$(find-pkg-share hermes_urdf)/config/rviz/config.rviz']
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz2
    ])
