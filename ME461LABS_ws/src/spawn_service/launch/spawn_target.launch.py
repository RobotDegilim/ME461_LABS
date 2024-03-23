from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    params_file_path = os.path.join(get_package_share_directory('spawn_service'), 'params', 'spawn_targets.yaml')

    spawn_target_node = Node(
        package='spawn_service',
        executable='spawn_targets_service',
        name='spawn_targets_service',
        parameters=[params_file_path],
        output='screen'
    )   

    ld.add_action(spawn_target_node)
    return ld
