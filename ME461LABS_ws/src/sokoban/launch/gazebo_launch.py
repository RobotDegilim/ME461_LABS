import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled

    package_name='sokoban' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), # launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    # world = os.path.join(get_package_share_directory(package_name), "worlds", "empty.world")

    # # set up gazebo and ros communication interface
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
    #         "/gazebo.launch.py",
    #     ]),
    #     launch_arguments={
    #         "world": world,
    #         "verbose": "true",
    #     }.items()
    # )
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    #     )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch', 'empty_world.launch.py'
        )])
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity_2 = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'sokoban/robot_description',
                                   '-entity', 'sokoban'],
                        output='screen')


    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity_2,
    ])
