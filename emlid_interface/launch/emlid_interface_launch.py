import os
import yaml
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    file_path = os.path.expanduser("~/robot_param.yaml")
    with open(file_path, "r") as stream:
        try:
            data_loaded = yaml.safe_load(stream)
            robot_namespace = '/'+ data_loaded['robot_namespace']

        except yaml.YAMLError as exc:
            print(exc)

    return LaunchDescription([
        Node(
            package='emlid_interface',
            namespace=robot_namespace,
            executable='emlid_interface_node',
            name='emlid_interface',
            parameters=[
                {'baud_rate': 57600},
            ],
            remappings=[
                ('rtk/fix', 'sensors/gps/fix'),
                ('rtk/utm', 'sensors/gps/utm'),
            ]
        ),
    ])
