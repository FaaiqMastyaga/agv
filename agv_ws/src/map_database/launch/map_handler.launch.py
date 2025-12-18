import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the YAML parameter file
    config = os.path.join(
        get_package_share_directory('map_database'),
    )

    return LaunchDescription([
        Node(
            package='map_database',
            executable='map_handler_node',
            name='map_handler',
            output='screen',
        ),
    ])