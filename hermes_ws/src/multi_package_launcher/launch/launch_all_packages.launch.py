# gps_publisher_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare a launch argument for namespace (optional)
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all nodes'
    )

    # ICM IMU Node
    icm_imu_node = Node(
        package='icm20948_publisher',
        executable='icm_imu',
        name='icm_imu_node',
        output='screen',
        namespace=LaunchConfiguration('namespace')
    )

    # IMU Filter Launch File
    imu_filter_share = os.path.expanduser('~/imu_tools/imu_filter_madgwick')
    imu_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(imu_filter_share, 'launch', 'imu_filter.launch.py')
        ]),
        launch_arguments={'namespace': LaunchConfiguration('namespace')}.items()
    )

    # Odometry Publisher Node
    odometry_publisher_node = Node(
        package='odometry_publisher',
        executable='odometry_publisher_node',
        name='odometry_publisher_node',
        output='screen',
        namespace=LaunchConfiguration('namespace')
    )

    # GPS Publisher Node
    gps_publisher_node = Node(
        package='gps_publisher',
        executable='gps_publisher_node',  # Ensure this matches your executable name
        name='gps_publisher_node',
        output='screen',
        namespace=LaunchConfiguration('namespace'),

    )

    # Assemble all actions into the launch description
    return LaunchDescription([
        namespace_arg,
        icm_imu_node,
        imu_filter_launch,
        odometry_publisher_node,
        gps_publisher_node,  # Add the GPS Publisher Node
    ])

