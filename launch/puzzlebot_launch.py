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
        get_package_share_directory('reto_sem_3'),
        'urdf',
        urdf_file_name)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Define the static transform from world >> map
    static_transform_node = Node(
                            package='tf2_ros',
                            executable='static_transform_publisher',
                            arguments = ['--x', '0', '--y', '0', '--z', '0.0',
                                        '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                                        '--frame-id', 'world', '--child-frame-id', 'map']
                            )
    
    # Define the static transform from map >> odom
    static_transform_node_2 = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '2', '--y', '1', '--z', '0.0',
                                            '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'map', '--child-frame-id', 'odom']
                                )
    
    # Define launch rviz config node
    rviz_config = os.path.join(
                            get_package_share_directory('reto_sem_3'),
                            'rviz',
                            'puzzlebot.rviz'
                            )
    
    # Define launch rviz node
    rviz_node = Node(name='rviz',
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config],
                    on_exit=Shutdown(),
                    )
    
    # Define launch rqt_tf_tree node
    rqt_tf_tree_node = Node(name='rqt_tf_tree',
                    package='rqt_tf_tree',
                    executable='rqt_tf_tree'
                    )
    
    
    #----------Start of robot 1 initialization----------------

    # Define puzzlebot node robot 1
    puzzlebot_node_1 = Node(name="puzzlebot",
                            package='reto_sem_3',
                            executable='puzzle_bot',
                            namespace='robot_1',
                            parameters=[{'initial_pos_x': -1.0,
                                        'initial_pos_y': 0.0,
                                        'initial_pos_z': 0.0,
                                        'initial_pos_yaw': 1.57,
                                        'initial_pos_pitch': 0.0,
                                        'initial_pos_roll': 0.0,
                                        'odom_frame': 'odom'}]
                            )
    
    # Define puzzlebot_sim node robot 1
    puzzlebot_sim_node_1 = Node(name="puzzlebot_sim",
                            package='reto_sem_3',
                            executable='puzzlebot_sim',
                            namespace='robot_1'
                            )
    
    # Define puzzlebot_control node robot 1
    puzzlebot_control_node_1 = Node(name="controller_node",
                            package='reto_sem_3',
                            executable='control',
                            namespace='robot_1',
                            parameters=[{'number_of_points': 4}] 
                            )
    
    # Define joint_state_publisher node for robot 1
    joint_state_publisher_node_1 = Node(
                                package='joint_state_publisher_gui',
                                executable='joint_state_publisher_gui',
                                output='screen',
                                namespace='robot_1'
                            )
    
    # Define robot_state_publisher node for robot 1
    robot_state_pub_node_1 = Node(
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            name='robot_state_publisher',
                            output='screen',
                            parameters=[{'frame_prefix': 'robot_1/','robot_description': robot_desc}],
                            arguments=[urdf],
                            namespace='robot_1'
                            )
    
    #----------End of robot 1 initialization-------------------
    

    #----------Start of robot 2 initialization----------------

    # Define puzzlebot node robot 2
    puzzlebot_node_2 = Node(name="puzzlebot",
                            package='reto_sem_3',
                            executable='puzzle_bot',
                            namespace='robot_2',
                            parameters=[{'initial_pos_x': 1.0,
                                        'initial_pos_y': 0.0,
                                        'initial_pos_z': 0.0,
                                        'initial_pos_yaw': 1.57,
                                        'initial_pos_pitch': 0.0,
                                        'initial_pos_roll': 0.0,
                                        'odom_frame': 'odom'}]
                            )
    
    # Define puzzlebot_sim node robot 2
    puzzlebot_sim_node_2 = Node(name="puzzlebot_sim",
                            package='reto_sem_3',
                            executable='puzzlebot_sim',
                            namespace='robot_2'
                            )
    
    # Define puzzlebot_control node robot 2
    puzzlebot_control_node_2 = Node(name="controller_node",
                            package='reto_sem_3',
                            executable='control',
                            namespace='robot_2',
                            parameters=[{'number_of_points': 4}] 
                            )
    
    # Define joint_state_publisher node for robot 2
    joint_state_publisher_node_2 = Node(
                                package='joint_state_publisher_gui',
                                executable='joint_state_publisher_gui',
                                output='screen',
                                namespace='robot_2'
                            )
    
    # Define robot_state_publisher node for robot 2
    robot_state_pub_node_2 = Node(
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            name='robot_state_publisher',
                            output='screen',
                            parameters=[{'frame_prefix': 'robot_2/','robot_description': robot_desc}],
                            arguments=[urdf],
                            namespace='robot_2'
                            )
    
    #----------End of robot 2 initialization-------------------

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
                        ) for node in [robot_state_pub_node_1, puzzlebot_node_1, robot_state_pub_node_2, puzzlebot_node_2]
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

    l_d = LaunchDescription([static_transform_node,static_transform_node_2, rviz_node, rqt_tf_tree_node, shutdown_log, *shutdown_on_exit, 
                             puzzlebot_control_node_1, puzzlebot_sim_node_1, puzzlebot_node_1, robot_state_pub_node_1, 
                             puzzlebot_control_node_2, puzzlebot_sim_node_2, puzzlebot_node_2, robot_state_pub_node_2])

    return l_d