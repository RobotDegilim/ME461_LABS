import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('sokoban'))
    cam_xacro_file = os.path.join(pkg_path,'urdf','sokoban.urdf.xacro')
     
    # Create a robot_state_publisher node
    cam_params = {'robot_description': xacro.process_file(cam_xacro_file).toxml(), 'use_sim_time': use_sim_time}
    cam_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[cam_params],
        remappings=[
                ('/robot_description', '/cam_description')
            ]
    )

    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        'turtlebot3_waffle.urdf')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
        
    turtle_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
        )
    
    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),

        cam_state_publisher,
        turtle_state_publisher,
    ])
