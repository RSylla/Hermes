import os


from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():

    package_name='hermes_urdf' 

    

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )
    
    

    #joystick = IncludeLaunchDescription(
    #            PythonLaunchDescriptionSource([os.path.join(
    #                get_package_share_directory(package_name),'launch','joystick.launch.py'
    #            )]), launch_arguments={'use_sim_time': 'true'}.items()
    #)

    #twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    #twist_mux = Node(
    #        package="twist_mux",
    #        executable="twist_mux",
    #        parameters=[twist_mux_params, {'use_sim_time': True}],
    #        remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    #    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'urdf','gazebo_hermes_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    #launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
    output='screen',
    additional_env={'RCUTILS_LOGGING_BUFFERED_STREAM': '1', 'RCUTILS_CONSOLE_OUTPUT_FORMAT': '{message}'}
    )



    #diff_drive_spawner = Node(
    #    package="controller_manager",
    #    executable="spawner.py",
    #    arguments=["diff_cont"],
    #)

    #joint_broad_spawner = Node(
    #    package="controller_manager",
    #    executable="spawner.py",
    #    arguments=["joint_broad"],
    #)


    # Code for delaying a node (I haven't tested how effective it is)
    # 
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner

    plane_model_path = os.path.join(get_package_share_directory(package_name),'urdf', 'my_world.sdf')

    # Command to spawn the plane model
    spawn_plane = ExecuteProcess(
        cmd=['gz', 'model', '--spawn-file', plane_model_path, '--model-name', 'my_plane'],
        output='screen'
    )


    # Launch them all!
    return LaunchDescription([
        rsp,
        #joystick,
        #twist_mux,
        gazebo,
        spawn_entity,
        #diff_drive_spawner,
        #joint_broad_spawner,
        spawn_plane,

    ])
