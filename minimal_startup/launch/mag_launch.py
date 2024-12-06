from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unitree_ros_bridge',
            node_namespace='foxtrot',
            node_executable='unitree_ros_bridge_server_node',
            node_name='unitree_ros_bridge_server_node',
            parameters=[
            ]
        ),
        Node(
            package='heading_controller',
            node_namespace='foxtrot',
            node_executable='heading_controller_node',
            node_name='heading_controller_node',
            parameters=[
                {'kp':1.0},
                {'kd':0.1},
                {'ki':0.01},
            ]
        ),
    ])
