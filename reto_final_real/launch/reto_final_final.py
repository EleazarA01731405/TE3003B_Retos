from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='reto_final_real',
            executable='aruco_create',  # camera_calib.py entry point
            name='camera_calib'
        ),
        Node(
            package='reto_final_real',
            executable='aruco_detect_node',  # aruco_detect_node.py entry point
            name='aruco_detect_node'
        ),
        Node(
            package='reto_final_real',
            executable='aruco_odom',  # odom.py entry point
            name='odom'
        ),
        Node(
            package='reto_final_real',
            executable='aruco_control',  # control.py entry point
            name='control'
        ), 
        Node(
            package='reto_final_real',
            executable='wall_follower',  # wall_follower.py entry point
            name='wall_follower'
        )
    ])