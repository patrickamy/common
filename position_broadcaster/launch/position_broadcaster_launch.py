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
            robot_name = data_loaded['robot_name']
            origin_easting = data_loaded['origin_easting']
            origin_northing = data_loaded['origin_northing']
            origin_rotation = data_loaded['origin_rotation']

        except yaml.YAMLError as exc:
            print(exc)

    return LaunchDescription([
        Node(
            package='position_broadcaster',
            namespace=robot_namespace,
            executable='position_broadcaster_node',
            name='position_broadcaster',
            parameters=[
                {'origin_easting': origin_easting},
                {'origin_northing': origin_northing},
                {'origin_rotation': origin_rotation},
                {'vehicle_name': robot_name},
            ],
            remappings=[
                ('imu/data', 'sensors/imu_0/data'),
                ('rtk/utm', 'sensors/gps/utm'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        ),
    ])
