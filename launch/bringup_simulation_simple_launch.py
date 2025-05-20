import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # -----------------------------------------------------------------------------
    #                          SIMULATION CONFIGURATION
    # -----------------------------------------------------------------------------
    
    # Name of the Gazebo world to load
    world = 'obstacle_avoidance_1.world'

    # General Gazebo settings
    pause = 'false'           # Start Gazebo in paused state
    verbosity = '4'           # Gazebo log verbosity level
    use_sim_time = 'true'     # Enable use of simulated clock (for ROS time sync)

    # Robot configurations (can be extended or loaded from a JSON file in future)
    robot_config_list = [
        {
            'name': '',
            'type': 'puzzlebot_jetson_lidar_ed',
            'x': 1.45, 'y': -3.5, 'yaw': 1.57,
            'lidar_frame': 'laser_frame',
            'camera_frame': 'camera_link_optical',
            'tof_frame': 'tof_link'
        }
    ]

    # -----------------------------------------------------------------------------
    #                         LOAD GAZEBO WORLD
    # -----------------------------------------------------------------------------

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('puzzlebot_gazebo'),
                'launch',
                'gazebo_world_launch.py'
            )
        ),
        launch_arguments={
            'world': world,
            'pause': pause,
            'verbosity': verbosity
        }.items()
    )

    # -----------------------------------------------------------------------------
    #                       SPAWN EACH ROBOT DYNAMICALLY
    # -----------------------------------------------------------------------------

    robot_launches = []
    for robot in robot_config_list:
        robot_name   = robot['name']
        robot_type   = robot['type']
        x            = str(robot.get('x', 0.0))
        y            = str(robot.get('y', 0.0))
        yaw          = str(robot.get('yaw', 0.0))
        lidar_frame  = robot.get('lidar_frame', 'laser_frame')
        camera_frame = robot.get('camera_frame', 'camera_link_optical')
        tof_frame    = robot.get('tof_frame', 'tof_link')
        prefix = f'{robot_name}/' if robot_name != '' else ''

        # Each robot is launched using the shared puzzlebot launch file
        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('puzzlebot_gazebo'),
                    'launch',
                    'gazebo_puzzlebot_launch.py'
                )
            ),
            launch_arguments={
                'robot': robot_type,
                'robot_name': robot_name,
                'x': x,
                'y': y,
                'yaw': yaw,
                'prefix': prefix,
                'lidar_frame': lidar_frame,
                'camera_frame': camera_frame,
                'tof_frame': tof_frame,
                'use_sim_time': use_sim_time
            }.items()
        )

        robot_launches.append(robot_launch)

    puzzlebot_node = Node(
                        name="bug0_algorithm",
                        package='reto_sem_6',
                        executable='puzzlebot_gazebo',
                        parameters=[{'use_sim_time': True}]
                        )
    
    odom_node = Node(
                    name="odom",
                    package='reto_sem_6',
                    executable='odom'
                    )
    
    puzzle_bot = Node(
                    name="puzzle_bot",
                    package='reto_sem_6',
                    executable='puzzle_bot'
                    )
    
    puzzlebot_sim = Node(
                    name="puzzlebot_sim",
                    package='reto_sem_6',
                    executable='puzzlebot_sim'
                    )
    # -----------------------------------------------------------------------------
    #                          BUILD FINAL LAUNCH DESCRIPTION
    # -----------------------------------------------------------------------------

    return LaunchDescription([
        gazebo_launch,
        *robot_launches,
        puzzlebot_node,
        odom_node
    ])