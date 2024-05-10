from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Setup Gazebo environment
    source_gazebo_cmd = ExecuteProcess(
        cmd=['bash', '-c', 'source /usr/share/gazebo/setup.bash && exec "$@"', 'dummy', 'gazebo'],
        output='screen'
    )

    # Gazebo launch
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    gazebo_launch_file = os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(get_package_share_directory('new_arm_gazebo'), 'worlds', 'new_arm_empty.world'), description='SDF world file'),
        source_gazebo_cmd,
        gazebo
    ])

