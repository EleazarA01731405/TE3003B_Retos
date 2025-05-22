import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Define wall_follower_node
    wall_follower_node = Node(
        name="Wall_Follower",
        package='reto_final_real',
        executable='wall_follower'
    )

    return LaunchDescription([
        wall_follower_node
    ])