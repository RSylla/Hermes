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