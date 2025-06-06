import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import  EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LocalSubstitution


def generate_launch_description():
    # -----------------------------------------------------------------------------
    #                          SIMULATION CONFIGURATION
    # -----------------------------------------------------------------------------
    
    # Name of the Gazebo world to load
    world = 'obstacle_avoidance_2.world'
    #'obstacle_avoidance_1.world'

    # General Gazebo settings
    pause = 'false'           # Start Gazebo in paused state
    verbosity = '4'           # Gazebo log verbosity level
    use_sim_time = 'true'     # Enable use of simulated clock (for ROS time sync)

    # Robot configurations (can be extended or loaded from a JSON file in future)
    robot_config_list = [
        {
            'name': '',
            'type': 'puzzlebot_jetson_lidar_ed',
            'x': -3, 'y': 1.0, 'yaw': -1.5708, #Posición inicial del robot
            #'x': -1.0, 'y': -.5, 'yaw': 0.0, #pa pruebas
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
                        package='reto_final_simulacion',
                        executable='puzzlebot_gazebo',
                        parameters=[{'use_sim_time': True}]
                        )
    
    odom_node = Node(
                    name="odom",
                    package='reto_final_simulacion',
                    executable='odom'
                    )
    
    aruco_node = Node(
                    name="aruco",
                    package='reto_final_simulacion',
                    executable='aruco'
                    )
    
    puzzlebot_sim = Node(
                    name="puzzlebot_sim",
                    package='reto_final_simulacion',
                    executable='puzzlebot_sim'
                    )
    
    rviz_config = os.path.join(
                            get_package_share_directory('reto_final_simulacion'),
                            'rviz',
                            'rviz_sim_reto.rviz'
                            )
    
    rviz_node = Node(name='rviz',
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config],
                    on_exit=Shutdown(),
                    )
    
    shutdown_on_exit = [RegisterEventHandler(
                            OnProcessExit(
                                target_action=node,
                                on_exit=[
                                    LogInfo(msg=(EnvironmentVariable(name='USER'),
                                            ' exited the node')),
                                    EmitEvent(event=Shutdown(
                                        reason='Node Exited'))
                                ]
                            )
                        ) for node in [aruco_node, puzzlebot_sim, odom_node, puzzlebot_node]
                    ]
 

    # Ensure full shutdown when SIGINT (Ctrl+C) is received
    shutdown_log = RegisterEventHandler(
                                    OnShutdown(
                                        on_shutdown=[LogInfo(
                                            msg=['Launch was asked to shutdown: ',
                                                LocalSubstitution('event.reason')]
                                        )]
                                    )
                                )
    # -----------------------------------------------------------------------------
    #                          BUILD FINAL LAUNCH DESCRIPTION
    # -----------------------------------------------------------------------------

    return LaunchDescription([
        gazebo_launch,
        *robot_launches,
        puzzlebot_node,
        odom_node,
        puzzlebot_sim,
        rviz_node,
        shutdown_log,
        *shutdown_on_exit,
        aruco_node
    ])