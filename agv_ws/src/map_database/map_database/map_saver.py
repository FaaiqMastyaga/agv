import rclpy
from rclpy.node import Node
from aruco_msgs.msg import Marker, MarkerArray
from map_database_interfaces.srv import SaveMap
from tf_transformations import euler_from_quaternion
import sqlite3
import os
import atexit

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')

        # Declare parameter
        self.declare_parameter('database_path', '~/agv/agv_data/map_data.db')
        self.db_path = os.path.expanduser(self.get_parameter('database_path').get_parameter_value().string_value)
        self.db_connection = None
        self.connect_to_db()

        # Service server
        self.map_saver_server_ = self.create_service(SaveMap, '/map_database/save_map', self.handleSaveMap)

        self.get_logger().info('Map Saver node has been started.')

    def connect_to_db(self):
        try:
            self.db_connection = sqlite3.connect(self.db_path, check_same_thread=False)
            self.db_connection.row_factory = sqlite3.Row
            self.get_logger().info('Successfully connected to SQLite database.')

            self.setup_database_schema()

            atexit.register(self.close_db_connection)

        except sqlite3.Error as e:
            self.get_logger().error(f"Failed to connect or set up database at {self.db_path}: {e}")
            self.db_connection = None
    
    def close_db_connection(self):
        if self.db_connection:
            self.db_connection.close()
            self.get_logger().info('Closed database connection.')

    def setup_database_schema(self):
        cursor = self.db_connection.cursor()

        # Create Maps Table if not exist
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS Maps (
                map_id INTEGER PRIMARY KEY AUTOINCREMENT,
                map_name TEXT NOT NULL UNIQUE,
                marker_dictionary TEXT NOT NULL,
                marker_size FLOAT NOT NULL, 
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
        """)

        # Create Markers Table if not exist
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS Markers (
                marker_pk INTEGER PRIMARY KEY AUTOINCREMENT,
                map_id INTEGER NOT NULL,
                marker_id INTEGER NOT NULL,
                x_coord FLOAT NOT NULL,
                y_coord FLOAT NOT NULL,
                orientation FLOAT NOT NULL,
                        
                FOREIGN KEY (map_id) REFERENCES Maps (map_id)
            );
        """)

        self.db_connection.commit()
        self.get_logger().info("Database schema checked and ready.")

    def handleSaveMap(self, request, response):
        map_name = request.map_name
        marker_dictionary = request.marker_dictionary
        marker_size = request.marker_size
        map_data = request.map_data

        response.success = False
        response.message = ""
        response.new_map_id = -1

        if not self.db_connection:
            response.message = "Database connection is unavailable."
            self.get_logger().error(response.message)
            return response

        cursor = self.db_connection.cursor()
        map_id = -1

        try:
            # Check if map_name exist on Maps Table
            cursor.execute("SELECT map_id FROM Maps WHERE map_name = ?", (map_name,))
            existing_record = cursor.fetchone()

            if existing_record:
                # UPDATE (Overwrite) Maps Table
                map_id = existing_record['map_id']
                cursor.execute("UPDATE Maps SET marker_dictionary = ?, marker_size = ?, created_at = CURRENT_TIMESTAMP WHERE map_id = ?", (marker_dictionary, marker_size, map_id,))
                self.get_logger().info(f"Updating existing map record: {map_name}")
            else:
                # INSERT (Create new record) to Maps Table
                cursor.execute("INSERT INTO Maps (map_name, marker_dictionary, marker_size) VALUES (?, ?, ?)", (map_name, marker_dictionary, marker_size,))
                map_id = cursor.lastrowid
                self.get_logger().info(f"Inserting new map record: {map_name} (ID: {map_id})")

            response.new_map_id = map_id

            # CLEAR and INSERT Markers to Markers Table
            # Delete old markers (if exist)
            cursor.execute("DELETE FROM Markers WHERE map_id = ?", (map_id,))

            # Insert new markers
            marker_data_list = []
            for marker in map_data.markers:
                # Convert quaternion to euler
                quaternion = [
                    marker.pose.pose.orientation.x,
                    marker.pose.pose.orientation.y,
                    marker.pose.pose.orientation.z,
                    marker.pose.pose.orientation.w,
                ]
                (roll, pitch, yaw) = euler_from_quaternion(quaternion)
                
                # Store data 
                marker_data_list.append((
                    map_id,
                    marker.id,
                    marker.pose.pose.position.x,
                    marker.pose.pose.position.y,
                    yaw,
                ))

            if marker_data_list:
                cursor.executemany("INSERT INTO Markers (map_id, marker_id, x_coord, y_coord, orientation) VALUES (?, ?, ?, ?, ?)", marker_data_list)

            self.db_connection.commit()

            response.success = True
            response.message = f"Map '{map_name}' saved/updated successfully. Marker count: {len(marker_data_list)}."

        except sqlite3.Error as e:
            self.db_connection.rollback()
            response.message = f"Database Transaction Error: {e}"
            self.get_logger().error(response.message)

        except Exception as e:
            response.message = f"Unexpected Error processing map data: {e}"
            self.get_logger().error(response.message)

        return response

def main(args=None):
    rclpy.init(args=args)
    map_saver = MapSaver()
    rclpy.spin(map_saver)
    map_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()