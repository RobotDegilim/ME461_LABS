import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    object_name_arg = launch.actions.DeclareLaunchArgument(
        'object_name', default_value='Donut',
        description='Name of the object to spawn')
    
    number_of_boxes_arg = launch.actions.DeclareLaunchArgument(
        'number_of_target', default_value='1',
        description='Number of targets to spawn')

    spawn_target_node = Node(
        package='spawn_service',
        executable='spawn_targets_service',
        name='spawn_targets_service',
        parameters=[{
            'object_name': launch.substitutions.LaunchConfiguration('object_name'),
            'number_of_target': launch.substitutions.LaunchConfiguration('number_of_target')
        }],
        output='screen'
    )   

    ld.add_action(object_name_arg)
    ld.add_action(number_of_boxes_arg)
    ld.add_action(spawn_target_node)
    return ld
