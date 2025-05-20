import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    wall_follower_node = Node(
                        name="Wall_Follower",
                        package='reto_real_final',
                        executable='wall_follower'
                        )

    return LaunchDescription([
        wall_follower_node
    ])