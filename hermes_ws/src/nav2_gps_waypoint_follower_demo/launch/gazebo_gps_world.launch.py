
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    gps_wpf_dir = get_package_share_directory(
        "nav2_gps_waypoint_follower_demo")
    launch_dir = os.path.join(gps_wpf_dir, 'launch')
    world = os.path.join(gps_wpf_dir, "worlds", "sonoma_raceway.world")

    urdf = os.path.join(gps_wpf_dir, 'urdf', 'turtlebot3_waffle_gps.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    models_dir = os.path.join(gps_wpf_dir, "models")
    models_dir += os.pathsep + \
        f"/opt/ros/{os.getenv('ROS_DISTRO')}/share/turtlebot3_gazebo/models"
    set_gazebo_model_path_cmd = None

    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path = os.environ['GAZEBO_MODEL_PATH'] + \
            os.pathsep + models_dir
        set_gazebo_model_path_cmd = SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH", gazebo_model_path)
    else:
        set_gazebo_model_path_cmd = SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH", models_dir)



    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[launch_dir], output='both')

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        cwd=[launch_dir], output='both')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set gazebo up to find models properly
    ld.add_action(set_gazebo_model_path_cmd)
    ld.add_action(set_tb3_model_cmd)

    # simulator launch
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # robot state publisher launch
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
