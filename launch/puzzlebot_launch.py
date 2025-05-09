import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import  EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LocalSubstitution


def generate_launch_description():

    urdf_file_name = 'puzzle_bot.urdf'
    urdf = os.path.join(
        get_package_share_directory('reto_sem_6'),
        'urdf',
        urdf_file_name)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Define the static transform world <-- map
    stf_node_0 = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '0', '--y', '0', '--z', '0.0',
                                            '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'world', '--child-frame-id', 'map']
                                )

    # Define the static transform map <-- odom
    stf_node_1 = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '2', '--y', '1', '--z', '0.0',
                                            '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'map', '--child-frame-id', 'odom']
                                )
    
    # Load rviz 2 config file
    rviz_config = os.path.join(
                            get_package_share_directory('reto_sem_6'),
                            'rviz',
                            'config.rviz'
                            )
    
    # Launch rviz 2
    rviz_node = Node(name='rviz',
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config],
                    on_exit=Shutdown(),
                    )
    
    # Launch tf tree
    rqt_tf_tree_node = Node(name='rqt_tf_tree',
                    package='rqt_tf_tree',
                    executable='rqt_tf_tree'
                    )

    #--------------------------Start of robot 0----------------------------
    # Launch puzzlebot
    puzzlebot_node_0 = Node(
                        name="puzzlebot",
                        package='reto_sem_6',
                        executable='puzzle_bot',
                        namespace='r0',
                        parameters=[{'initial_pos_x':0.0,       # Intitial position for robot <#> on x
                                     'initial_pos_y':0.0,       # Intitial position for robot <#> on y
                                     'initial_pos_z':0.0,       # Intitial position for robot <#> on y
                                     'initial_pos_roll':0.0,    # Intitial orientation for robot <#> on roll
                                     'initial_pos_pitch':0.0,   # Intitial orientation for robot <#> on pitch
                                     'initial_pos_yaw':0.0,     # Intitial orientation for robot <#> on yaw
                                     'odom_frame':'odom'}]      # Robot <#> initial parent frame
                            )
    
    # Launch path 
    puzzlebot_sim_node_0 = Node(
                            name="puzzlebot_sim",
                            package='reto_sem_6',
                            executable='puzzlebot_sim',
                            namespace="r0"
                            )
    
    # Launch control node
    puzzlebot_control_node_0 = Node(
                            name="controller_node",
                            package='reto_sem_6',
                            executable='control',
                            namespace="r0"
                            )
    
    # Launch point generator 
    point_gen_node_0 = Node(
                            name="point_gen",
                            package='reto_sem_6',
                            executable="point_gen",
                            namespace="r0",
                            parameters=[
                                {'gen_type': 0}, 
                                {'n_points': 4},
                                {'point': [0.0, 0.0]},
                            ]
                          )
    
    robot_state_pub_node_0 = Node(
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            name='robot_state_publisher',
                            output='screen',
                            parameters=[{'robot_description': robot_desc}],
                            arguments=[urdf],
                            namespace="r0"
                            )
    #-------------------------------------------------------------------------

    #-----------------------------Start of robot 2----------------------------
    # Launch puzzlebot
    puzzlebot_node_1 = Node(
                        name="puzzlebot",
                        package='reto_sem_6',
                        executable='puzzle_bot',
                        namespace='r1',
                        parameters=[{'initial_pos_x':0.0,       # Intitial position for robot <#> on x
                                     'initial_pos_y':0.0,       # Intitial position for robot <#> on y
                                     'initial_pos_z':0.0,       # Intitial position for robot <#> on y
                                     'initial_pos_roll':0.0,    # Intitial orientation for robot <#> on roll
                                     'initial_pos_pitch':0.0,   # Intitial orientation for robot <#> on pitch
                                     'initial_pos_yaw':0.0,     # Intitial orientation for robot <#> on yaw
                                     'odom_frame':'odom'}]      # Robot <#> initial parent frame
                            )
    
    # Launch path 
    puzzlebot_sim_node_1 = Node(
                            name="puzzlebot_sim",
                            package='reto_sem_6',
                            executable='puzzlebot_sim',
                            namespace="r1"
                            )
    
    # Launch control node
    puzzlebot_control_node_1 = Node(
                            name="controller_node",
                            package='reto_sem_6',
                            executable='control',
                            namespace="r1"
                            )
    
    # Launch point generator 
    point_gen_node_1 = Node(
                            name="point_gen",
                            package='reto_sem_6',
                            executable="point_gen",
                            namespace="r1",
                            parameters=[
                                {'gen_type': 0}, 
                                {'n_points': 4},
                                {'point': [0.0, 0.0]},
                            ]
                          )
    
    robot_state_pub_node_1 = Node(
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            name='robot_state_publisher',
                            output='screen',
                            parameters=[{'robot_description': robot_desc}],
                            arguments=[urdf],
                            namespace="r1"
                            )
    #-------------------------------------------------------------------------
    
    #Shutdown node on exit condition
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
                        ) for node in [robot_state_pub_node_0, puzzlebot_node_0]
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
    
    # Launch 
    l_d = LaunchDescription([stf_node_0, stf_node_1, rviz_config, rviz_node, rqt_tf_tree_node, shutdown_log, *shutdown_on_exit,
                             puzzlebot_node_0, puzzlebot_sim_node_0, puzzlebot_control_node_0, point_gen_node_0, robot_state_pub_node_0,
                             puzzlebot_node_1, puzzlebot_sim_node_1, puzzlebot_control_node_1, point_gen_node_1, robot_state_pub_node_1])

    return l_d

    #   Launch logic
    #   stf_node_0          | First static transformation from world to map
    #   stf_node_1          | Second static transform from map to odom
    #   rviz_config         | Rviz2 confiduration file <name>.rviz
    #   rviz_node           | Rviz2 Launcher
    #   rqt_tf_tree_node    | Rqt TF Tree Launcher
    #   shutdown_log        | Shutdown on Rviz window closing
    #   *shutdown_on_exit   | Shutdown with Crtl + C
    
    #   Launch for each robot: 
    #   puzzlebot_node_<#>, puzzlebot_sim_node_<#>, puzzlebot_control_node_<#>, point_gen_node_<#>, robot_state_pub_node_<#>