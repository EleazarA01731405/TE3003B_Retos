from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
      
    static_transform_node_3 = Node(
                                name="static_tf",
                                package='tf_examples',
                                executable='static_tf'
                                )
    
    dynamic_transform_node = Node(
                                name="dyn_tf",
                                package='tf_examples',
                                executable='dyn_tf'
                                )
    
    transform_listener_node = Node(
                                name="tf_listener",
                                package='tf_examples',
                                executable='tf_listener',
                                output = 'screen'
                                )

    rviz_node = Node(name='rviz',
                    package='rviz2',
                    executable='rviz2'
                    )
    
    rqt_tf_tree_node = Node(name='rqt_tf_tree',
                    package='rqt_tf_tree',
                    executable='rqt_tf_tree'
                    )
    
    l_d = LaunchDescription([rqt_tf_tree_node, rviz_node, static_transform_node_3, dynamic_transform_node, transform_listener_node])

    return l_d