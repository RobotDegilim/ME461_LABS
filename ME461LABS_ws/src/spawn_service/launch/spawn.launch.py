import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    spawn_node = Node(
        package='spawn_service',
        executable='spawn_model',
        name='spawn_model',
        output='screen'
    )

    ld.add_action(spawn_node)
    return ld
