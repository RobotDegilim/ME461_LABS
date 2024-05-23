import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument



def generate_launch_description():
    
    declare_x_position = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')
    
    declare_world = DeclareLaunchArgument(
        name="world",
        default_value="empty.world",
        description='world to launch rhex in'
    )
    
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    world_sub = LaunchConfiguration("world")
    
    package_name = 'sokoban'    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Add the models directory to the Gazebo model path 
    models_dir = os.path.join(get_package_share_directory('sokoban'), 'models')
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', models_dir + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    world = PathJoinSubstitution([
                    FindPackageShare(package_name),
                    'worlds',
                    world_sub])
    
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','robot_state_publisher.launch.py'
                )]), 
    )
    
    # Get the urdf file
    model_folder = 'turtlebot3_waffle'
    turtlebot_urdf_path = os.path.join(
        get_package_share_directory('sokoban'),
        'models',
        model_folder,
        'model.sdf'
    )

    spawn_turtlebot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'waffle',
            '-file', turtlebot_urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    spawn_camera_box = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'cam_description',
                                   '-entity', 'sokoban'],
                        output='screen')


    # Launch them all!
    return LaunchDescription([
        declare_x_position,
        declare_y_position,
        declare_world,
        set_model_path,
        gzserver_cmd,
        gzclient_cmd,
        rsp,
        spawn_turtlebot,
        spawn_camera_box,                            
    ])



