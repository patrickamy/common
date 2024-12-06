import launch_ros.actions
import os
import yaml
import pathlib
import launch.actions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    file_path = os.path.expanduser("~/robot_param.yaml")
    with open(file_path, "r") as stream:
        try:
            data_loaded = yaml.safe_load(stream)
            robot_namespace = '/'+ data_loaded['robot_namespace']
        except yaml.YAMLError as exc:
            print(exc)

    return LaunchDescription([
    launch_ros.actions.Node(
            package='tf2_ros',
            namespace=robot_namespace,
            executable='static_transform_publisher',
            name='static_navsat_tf_publisher',
            arguments=["0.0","0.0","0.64","0.0","0.0","0.0", "base_link", "navsat_link"],
            parameters=[],
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                        ('/diagnostics', 'diagnostics'),
                        ],
           ),
   #launch_ros.actions.Node(
   #        package='tf2_ros',
   #        namespace=robot_namespace,
   #        executable='static_transform_publisher',
   #        name='static_park_tf_publisher',
   #        arguments=["-368305.0700699703","-3278357.100811787","0.0","0.0","0.0","-0.6021965548550632", "utm", "autonomy_park"],
   #        parameters=[],
   #        remappings=[('/tf', 'tf'),
   #                    ('/tf_static', 'tf_static'),
   #                    ('/diagnostics', 'diagnostics'),
   #                    ],
   #       ),
])
