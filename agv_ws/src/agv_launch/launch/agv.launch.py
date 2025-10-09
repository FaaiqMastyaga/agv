import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    agv_launch_dir = get_package_share_directory('agv_launch')
    vision_dir = get_package_share_directory('vision')
    localization_dir = get_package_share_directory('localization')
    map_database_dir = get_package_share_directory('map_database')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(agv_launch_dir, 'launch', 'web_communication.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(vision_dir, 'launch', 'detect_aruco.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(localization_dir, 'launch', 'aruco_localization.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(localization_dir, 'launch', 'map_handler.launch.py')
            )
        ),
    ])
