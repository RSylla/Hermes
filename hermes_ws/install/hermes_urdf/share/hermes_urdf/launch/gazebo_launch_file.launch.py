import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    robotName = 'hermes'
    namePackage = 'hermes_urdf'
    modelFile = 'hermes_model.urdf'
    worldFile = 'hermes_model.sdf'
    
    pathModelFile = os.path.join(get_package_share_directory(namePackage), 'urdf', modelFile)
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), 'urdf', worldFile)
    
    robotDescription = xacro.process_file(pathModelFile).toxml()
    
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'))
    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'world': pathWorldFile}.items())
    
    spawnModelNode = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic', 'robot_description', '-entity', robotName], output='screen')
    
    nodeRobotStatePublisher = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[{'robot_description': robotDescription, 'use_sim_time': True}])
    
    LaunchDescriptionObject = LaunchDescription()
    LaunchDescriptionObject.add_action(gazeboLaunch)
    LaunchDescriptionObject.add_action(spawnModelNode)
    LaunchDescriptionObject.add_action(nodeRobotStatePublisher)
    
    return LaunchDescriptionObject
    
    #error  ros2 launch hermes_urdf gazebo_launch_file.launch.py 
[INFO] [launch]: All log files can be found below /home/aleksmalm/.ros/log/2024-05-14-15-48-55-784938-aleksmalm-Latitude-5430-257349
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [gzserver-1]: process started with pid [257350]
[INFO] [gzclient-2]: process started with pid [257352]
[INFO] [spawn_entity.py-3]: process started with pid [257354]
[INFO] [robot_state_publisher-4]: process started with pid [257356]
[robot_state_publisher-4] [WARN] [1715690936.098460105] [kdl_parser]: The root link base_footprint has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
[robot_state_publisher-4] [INFO] [1715690936.098532138] [robot_state_publisher]: got segment base_footprint
[robot_state_publisher-4] [INFO] [1715690936.098572877] [robot_state_publisher]: got segment base_link
[robot_state_publisher-4] [INFO] [1715690936.098576372] [robot_state_publisher]: got segment front_left_wheel
[robot_state_publisher-4] [INFO] [1715690936.098578848] [robot_state_publisher]: got segment front_right_wheel
[robot_state_publisher-4] [INFO] [1715690936.098581258] [robot_state_publisher]: got segment ld19
[robot_state_publisher-4] [INFO] [1715690936.098583767] [robot_state_publisher]: got segment liigend
[robot_state_publisher-4] [INFO] [1715690936.098586099] [robot_state_publisher]: got segment rear_left_wheel
[robot_state_publisher-4] [INFO] [1715690936.098588566] [robot_state_publisher]: got segment rear_right_wheel
[spawn_entity.py-3] [INFO] [1715690936.346238899] [spawn_entity]: Spawn Entity started
[spawn_entity.py-3] [INFO] [1715690936.346431411] [spawn_entity]: Loading entity published on topic robot_description
[spawn_entity.py-3] /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/qos.py:307: UserWarning: DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL is deprecated. Use DurabilityPolicy.TRANSIENT_LOCAL instead.
[spawn_entity.py-3]   warnings.warn(
[spawn_entity.py-3] [INFO] [1715690936.347741109] [spawn_entity]: Waiting for entity xml on robot_description
[spawn_entity.py-3] [INFO] [1715690936.358605572] [spawn_entity]: Waiting for service /spawn_entity, timeout = 30
[spawn_entity.py-3] [INFO] [1715690936.358779446] [spawn_entity]: Waiting for service /spawn_entity
[spawn_entity.py-3] [INFO] [1715690936.862899474] [spawn_entity]: Calling service /spawn_entity
[spawn_entity.py-3] [INFO] [1715690937.008367263] [spawn_entity]: Spawn status: SpawnEntity: Successfully spawned entity [hermes]
[gzserver-1] Error [Element.cc:914] Missing element description for [left_joint]
[gzserver-1] Error [Element.cc:914] Missing element description for [right_joint]
[gzserver-1] [ERROR] [1715690937.019487093] [differential_drive_controller]: Inconsistent number of joints specified. Plugin will not work.
[INFO] [spawn_entity.py-3]: process has finished cleanly [pid 257354]
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[robot_state_publisher-4] [INFO] [1715690950.161368669] [rclcpp]: signal_handler(signum=2)
[INFO] [robot_state_publisher-4]: process has finished cleanly [pid 257356]
[INFO] [gzserver-1]: process has finished cleanly [pid 257350]
[INFO] [gzclient-2]: process has finished cleanly [pid 257352]

