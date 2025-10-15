import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    rosbridge_server_pkg = get_package_share_directory('rosbridge_server')
    web_video_server_pkg = get_package_share_directory('web_video_server')

    return LaunchDescription([
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(rosbridge_server_pkg, 'launch', 'rosbridge_websocket_launch.xml')
            )
        ),
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            parameters=[{'port': 8080}]
        ),
    ])
