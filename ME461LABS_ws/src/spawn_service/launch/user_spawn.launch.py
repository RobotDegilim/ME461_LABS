import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    object_name_arg = launch.actions.DeclareLaunchArgument(
        'object_name', default_value='Donut',
        description='Name of the object to spawn')
    
    x_position_arg = launch.actions.DeclareLaunchArgument(
        'x_position', default_value='0.0',
        description='X position of the object to spawn')
    
    y_position_arg = launch.actions.DeclareLaunchArgument(
        'y_position', default_value='0.0',
        description='Y position of the object to spawn')
    
    z_position_arg = launch.actions.DeclareLaunchArgument(
        'z_position', default_value='0.0',
        description='Z position of the object to spawn')
    
    user_spawn_node = Node(
        package='spawn_service',
        executable='user_spawn_service',
        name='user_spawn_service',
        parameters=[{
            'object_name': launch.substitutions.LaunchConfiguration('object_name'),
            'x_position': launch.substitutions.LaunchConfiguration('x_position'),
            'y_position': launch.substitutions.LaunchConfiguration('y_position'),
            'z_position': launch.substitutions.LaunchConfiguration('z_position')
        }],
        output='screen'
    )

    ld.add_action(object_name_arg)
    ld.add_action(x_position_arg)
    ld.add_action(y_position_arg)
    ld.add_action(z_position_arg)
    ld.add_action(user_spawn_node)
    return ld
