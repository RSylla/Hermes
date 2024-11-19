# gps_publisher_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare a launch argument for namespace (optional)
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all nodes'
    )

      # BNO055 IMU Node
    bno055_node = Node(
        package='bno055_driver',
        executable='bno055_node',
        name='bno055_node',
        output='screen',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'frame_id': 'imu_link',
            'bus_num': 1,
            'i2c_address': 0x28,
            'update_rate': 100.0
        }]
    )
    
    bno08x_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros2_bno08x_driver'),
                'launch',
                'bno08x.launch.py'
            ])
        ]),
        launch_arguments={'namespace': LaunchConfiguration('namespace')}.items()
    )


    # IMU Filter Launch File
    """ imu_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('imu_filter_madgwick'),
                'launch',
                'imu_filter.launch.py'
            ])
        ]),
        launch_arguments={'namespace': LaunchConfiguration('namespace')}.items()
    )  """

    # Odometry Publisher Node
    odometry_publisher_node = Node(
        package='odometry_publisher',
        executable='odometry_publisher_node',
        name='odometry_publisher_node',
        output='screen',
        namespace=LaunchConfiguration('namespace')
    ) 

    # GPS Publisher Node
    """  gps_publisher_node = Node(
        package='gps_publisher',
        executable='gps_publisher_node',
        name='gps_publisher_node',
        output='screen',
        namespace=LaunchConfiguration('namespace'),
    ) """

    # Wheel Joint Publisher Node
    """  wheel_joint_publisher_node = Node(
        package='wheel_joint_publisher_pkg',
        executable='wheel_joint_publisher',
        name='wheel_joint_publisher_node',
        output='screen',
        namespace=LaunchConfiguration('namespace'),
    )   """

    # Assemble all actions into the launch description
    return LaunchDescription([
        namespace_arg,
        #bno055_node,
        bno08x_launch,  
        #imu_filter_launch,
        odometry_publisher_node,
        #gps_publisher_node,
        #wheel_joint_publisher_node,
    ])
