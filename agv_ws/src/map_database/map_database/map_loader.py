import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from aruco_msgs.msg import Marker, MarkerArray
import sqlite3
import os

class MapLoader(Node):
    def __init__(self):
        super().__init__('map_loader')

        # Service Server
        self.map_loader_server_ = self.create_service(LoadMapService, '/map_database/load_map', self.loadMapCallback)
        self.map_getter_server_ = self.create_service(GetMapService, '/map_database/get_map')

        # Publisher
        self.map_signal_pub_ = self.create_publisher(String, '/map_database/map_update_signal', 10)

        self.get_logger().info('Map Loader node has been started.')

    def loadMapCallback(self, request, response):
        msg = String()
        self.map_signal_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    map_loader = MapLoader()
    rclpy.spin(map_loader)
    map_loader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()