import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include Packages
    minimal_startup = FindPackageShare('minimal_startup')

    # Declare launch files
    #launch_file_microstrain_imu = PathJoinSubstitution([
    #    minimal_startup, 'launch/include', 'microstrain_imu.launch.py'])

    #param_file_microstrain_imu = PathJoinSubstitution([
    #    minimal_startup, 'config', 'imu_0.yaml'])

    param_file_microstrain_imu = '/home/administrator/platform_ws/src/common/minimal_startup/config/imu_0.yaml'

    # Include launch files
    launch_microstrain_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('minimal_startup')),
            '/microstrain_imu.launch.py']),
        launch_arguments=
        [
            (
                'parameters'
                ,
                param_file_microstrain_imu
            )
            ,
            (
                'namespace'
                ,
                'a200_0706/sensors/imu_0'
            )
            ,
        ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_microstrain_imu)
    return ld
