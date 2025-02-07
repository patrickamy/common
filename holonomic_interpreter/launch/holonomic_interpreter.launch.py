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
            kp = data_loaded['kp']
            kd = data_loaded['kd']
            ki = data_loaded['ki']
            max_vel = data_loaded['max_vel']
            min_vel = data_loaded['min_vel']
            max_ang_acc = data_loaded['max_ang_acc']
            origin_rotation = data_loaded['origin_rotation']

        except yaml.YAMLError as exc:
            print(exc)

    return LaunchDescription([
        Node(
            package='holonomic_interpreter',
            namespace=robot_namespace,
            executable='holonomic_interpreter_node',
            name='holonomic_interpreter',
            parameters=[
            #    {'origin_rotation': origin_rotation},
            #    {'max_vel': max_vel},
            #    {'min_vel': min_vel},
            #    {'max_ang_acc': max_ang_acc},
            #    {'heading_kp': kp},
            #    {'heading_kd': kd},
            #    {'heading_ki': ki},
            ],
            remappings=[
                ('non_holo_cmd_vel', 'cmd_vel'),
                ('odom', 'odom_global'),
            #    ('holo_cmd_vel', 'other'),
            ]
        ),
    ])