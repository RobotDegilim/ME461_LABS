import os

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    ld = LaunchDescription()
    
    config = os.path.join(
        get_package_share_directory('spawn_objects'),
        'launch',
        'spawn_params.yaml'
    )

    spawn_box_node = Node(
        package='spawn_objects',
        executable='spawn_objects',
        name='object_spawner',
        parameters = [config],
        output='screen'
    )   

    ld.add_action(spawn_box_node)
    return ld
