import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    object_name_arg = launch.actions.DeclareLaunchArgument(
        'object_name', default_value='box',
        description='Name of the object to spawn')
    number_of_boxes_arg = launch.actions.DeclareLaunchArgument(
        'number_of_boxes', default_value='2',
        description='Number of boxes to spawn')
    
    spawn_box_node = Node(
        package='spawn_service',
        executable='spawn_service',
        name='spawn_service',
        parameters=[{
            'object_name': launch.substitutions.LaunchConfiguration('object_name'),
            'number_of_boxes': launch.substitutions.LaunchConfiguration('number_of_boxes')
        }],
        output='screen'
    )   

    ld.add_action(object_name_arg)
    ld.add_action(number_of_boxes_arg)
    ld.add_action(spawn_box_node)
    return ld
