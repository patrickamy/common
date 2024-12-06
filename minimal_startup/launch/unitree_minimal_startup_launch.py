from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='geodetic_conversion',
            node_namespace='foxtrot',
            node_executable='geo_conversion_node',
            node_name='geodetic_conversion_node',
            parameters=[
            ]
        ),
        # Node(
        #     package='wpt_control_xy',
        #     node_namespace='foxtrot',
        #     node_executable='wpt_control_node',
        #     node_name='wpt_control_node',
        #     parameters=[
        #     ]
        # ),
        Node(
            package='emlid_interface_tcp',
            node_namespace='foxtrot',
            node_executable='emlid_interface_node',
            node_name='emlid_interface_node',
            parameters=[
            ]
        ),
        Node(
            package='unitree_ros_bridge',
            node_namespace='foxtrot',
            node_executable='unitree_ros_bridge_server_node',
            node_name='unitree_ros_bridge_server_node',
            parameters=[
            ]
        ),
        Node(
            package='udp_velocity_interface',
            node_namespace='foxtrot',
            node_executable='udp_velocity_interface',
            node_name='unitree_ros_bridge_server_node',
            parameters=[
            ]
        ),
        Node(
            package='unitree_legged_real',
            node_namespace='foxtrot',
            node_executable='ros2_udp',
            node_name='ros2_udp',
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
