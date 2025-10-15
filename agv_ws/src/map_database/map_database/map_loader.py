import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from aruco_msgs.msg import Marker, MarkerArray
from map_database_interfaces.srv import LoadMap, GetMap, GetMapName
from tf_transformations import quaternion_from_euler
import numpy as np
import sqlite3
import os
import atexit

class MapLoader(Node):
    def __init__(self):
        super().__init__('map_loader')

        self.current_map = MarkerArray()
        self.marker_dictionary = ""
        self.marker_size = 0.0

        # Declare parameter
        self.declare_parameter('database_path', '~/agv/agv_data/map_data.db')
        self.db_path = os.path.expanduser(self.get_parameter('database_path').get_parameter_value().string_value)
        self.db_connection = None
        self.connect_to_db()

        # Service Server
        self.map_loader_server_ = self.create_service(LoadMap, '/map_database/load_map', self.loadMapCallback)
        self.map_getter_server_ = self.create_service(GetMap, '/map_database/get_map', self.getMapCallback)
        self.map_list_getter_server_ = self.create_service(GetMapName, '/map_database/get_map_name', self.getMapNameCallback)

        # Publisher
        self.map_signal_pub_ = self.create_publisher(String, '/map_database/map_update_signal', 10)

        self.get_logger().info(f'Map Loader node has been started. DB Path: {self.db_path}')

    def connect_to_db(self):
        try:
            self.db_connection = sqlite3.connect(self.db_path, check_same_thread=False)
            self.db_connection.row_factory = sqlite3.Row
            self.get_logger().info('Successfully connected to SQLite database.')

            atexit.register(self.close_db_connection)

        except sqlite3.Error as e:
            self.get_logger().error(f"Failed to connect to database at {self.db_path}:  {e}")

    def close_db_connection(self):
        if self.db_connection:
            self.db_connection.close()
            self.get_logger().info('Closed database connection.')

    def loadMapCallback(self, request, response):
        map_name = request.map_name

        response.success = False
        response.message = ""

        if not self.db_connection:
            response.message = "Database connection is not available."
            self.get_logger().error(response.message)
            return response
        
        # Initialize a temporary MarkerArray for new map
        new_map_array = MarkerArray()

        try:
            cursor = self.db_connection.cursor()

            # Find the map_id by name
            cursor.execute("SELECT map_id, marker_dictionary, marker_size FROM Maps WHERE map_name = ?", (map_name,))
            map_metadata = cursor.fetchone()

            if not map_metadata:
                response.message = f"Map name '{map_name}' not found in database."
                self.get_logger().warn(response.message)
                return response
            
            map_id = map_metadata['map_id']

            cursor.execute("SELECT marker_id, x_coord, y_coord, orientation FROM Markers WHERE map_id = ?", (map_id,))
            map_data = cursor.fetchall()

            if not map_data:
                response.message = f"Map ID {map_id} found, but contains no markers."
                self.get_logger().warn(response.message)
                return response

            # Process raw data into ROS MarkerArray
            for row in map_data:
                marker = Marker()
                marker.id = row['marker_id']
                marker.pose.pose.position.x = row['x_coord']
                marker.pose.pose.position.y = row['y_coord']
                marker.pose.pose.position.z = 0.0

                yaw_in_radians = row['orientation']
                quaternion = quaternion_from_euler(0, 0, yaw_in_radians)
                marker.pose.pose.orientation.x = quaternion[0]
                marker.pose.pose.orientation.y = quaternion[1]
                marker.pose.pose.orientation.z = quaternion[2]
                marker.pose.pose.orientation.w = quaternion[3]

                new_map_array.markers.append(marker)

            self.current_map = new_map_array
            self.marker_dictionary = map_metadata['marker_dictionary']
            self.marker_size = map_metadata['marker_size']

            map_signal_msg = String()
            map_signal_msg.data = f"MAP_UPDATE_SUCCESS: {map_name}"
            self.map_signal_pub_.publish(map_signal_msg)

            response.success = True
            response.message = f"Map '{map_name}' (ID: {map_id}) successfully loaded and published signal."
            self.get_logger().info(response.message)

        except sqlite3.Error as e:
            response.message = f"SQLite Database Error: {e}"
            self.get_logger().error(response.message)

        except Exception as e:
            response.message = f"Unexpected Error: {e}"
            self.get_logger().error(response.message)
        
        return response

    def getMapCallback(self, request, response):
        if self.current_map is not None and len(self.current_map.markers) > 0:
            response.map_data = self.current_map
            response.marker_dictionary = self.marker_dictionary
            response.marker_size = self.marker_size
            response.success = True
            response.message = f"Successfully retrieved map with {len(self.current_map.markers)} markers."
        else:
            response.success = False
            response.message = "No map data is currently cached in the loader node."

        return response
    
    def getMapNameCallback(self, request, response):
        response.success = False
        response.map_names = []

        if not self.db_connection:
            response.message = "Database connection is not available."
            self.get_logger().error(response.message)
            return response

        try:
            cursor = self.db_connection.cursor()

            # Query all map name
            cursor.execute("SELECT map_name FROM Maps ORDER BY map_name ASC")
            map_records = cursor.fetchall()

            response.map_names = [row['map_name'] for row in map_records]
            response.success = True
            response.message = f"Successfully retrieved {len(response.map_names)} map_names."
            self.get_logger().info(response.message)

        except sqlite3.Error as e:
            response.message = f"SQLite Database Error: {e}"
            self.get_logger().error(response.message)

        return response
    
def main(args=None):
    rclpy.init(args=args)
    map_loader = MapLoader()
    rclpy.spin(map_loader)
    map_loader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()