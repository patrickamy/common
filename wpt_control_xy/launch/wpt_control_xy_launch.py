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
            origin_rotation = data_loaded['origin_rotation']
            capture_radius = data_loaded['capture_radius']
            lead_distance = data_loaded['lead_distance']
            max_vel = data_loaded['max_vel']
            min_vel = data_loaded['min_vel']

        except yaml.YAMLError as exc:
            print(exc)

    return LaunchDescription([
        Node(
            package='wpt_control_xy',
            namespace=robot_namespace,
            executable='wpt_control_xy_node',
            name='wpt_control_xy',
            parameters=[
                {'origin_rotation': origin_rotation},
                {'capture_radius': capture_radius},
                {'lead_distance': lead_distance},
                {'max_vel': max_vel},
                {'min_vel': min_vel},
            ],
        ),
    ])
