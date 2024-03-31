import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument



def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    package_name='sokoban' #<--- CHANGE ME
    models_dir = os.path.join(get_package_share_directory('sokoban'), 'models')
    # Add the models directory to the Gazebo model path 
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', models_dir + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), # launch_arguments={'use_sim_time': 'true'}.items()
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'easymode_v1.world'
    )

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

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawning the turtlebot in the world
    # spawn_turtlebot_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
    #     ),
    #     launch_arguments={
    #         'x_pose': x_pose,
    #         'y_pose': y_pose
    #     }.items()
    # )
    # Get the urdf file
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('sokoban'),
        'models',
        model_folder,
        'model.sdf'
    )

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')
    
    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity_2 = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'sokoban/robot_description',
                                   '-entity', 'sokoban'],
                        output='screen')


    # Launch them all!
    return LaunchDescription([
        set_model_path,
        rsp,
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        declare_x_position_cmd,
        declare_y_position_cmd,
        start_gazebo_ros_spawner_cmd,
        spawn_entity_2,                            #spawn_turtlebot_cmd,
    ])



