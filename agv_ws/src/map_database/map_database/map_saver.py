import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from aruco_msgs.msg import Marker, MarkerArray
import sqlite3
import os

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')

        # Service server
        self.map_saver_server_ = self.create_service(SaveMapService, '/map_database/save_map')

        self.get_logger().info('Map Saver node has been started.')

def main(args=None):
    rclpy.init(args=args)
    map_saver = MapSaver()
    rclpy.spin(map_saver)
    map_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()