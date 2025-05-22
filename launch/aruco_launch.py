from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_create',
            executable='aruco_create',  # camera_calib.py entry point
            name='camera_calib'
        ),
        Node(
            package='aruco_create',
            executable='aruco_detect_node',  # aruco_detect_node.py entry point
            name='aruco_detect_node'
        ),
    ])