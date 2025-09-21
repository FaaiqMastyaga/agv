import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the YAML parameter file
    config = os.path.join(
        get_package_share_directory('vision'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='vision',
            executable='image_publisher_node',
            name='image_publisher',
            output='screen',
            parameters=[config]    
        ),
        Node(
            package='vision',
            executable='aruco_detector_node',
            name='aruco_detector',
            output='screen',
            parameters=[config]    
        ),
    ])