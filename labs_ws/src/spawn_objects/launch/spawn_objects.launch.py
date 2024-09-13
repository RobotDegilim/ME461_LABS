import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    ld = LaunchDescription()
    
    #* Declare Spawn From Json Launch Argument
    declare_spawn_from_json = DeclareLaunchArgument(
        'spawn_from_json',
        default_value='false',
        description='Spawn from JSON file if true. Else spawn objects randomly'
    )
    spawn_from_json = LaunchConfiguration('spawn_from_json')
    
    #* spawn_params.yaml parameter file
    config = os.path.join(
        get_package_share_directory('spawn_objects'),
        'launch',
        'spawn_params.yaml'
    )

    spawn_box_node = Node(
        package='spawn_objects',
        executable='spawn_objects',
        name='object_spawner',
        parameters = [{'spawn_from_json': spawn_from_json},
                        config],
        output='screen'
    )   

    ld.add_action(declare_spawn_from_json)
    ld.add_action(spawn_box_node)
    return ld
