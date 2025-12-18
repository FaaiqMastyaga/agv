import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from aruco_msgs.msg import Marker, MarkerArray
from map_database_interfaces.srv import LoadMap, SaveMap, RemoveMap, GetMap, GetMapName
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import sqlite3
import os
import atexit

class MapHandler(Node):
    def __init__(self):
        super().__init__('map_handler')

        # 1. Setup database path & ensure directory exists
        home_dir = os.path.expanduser('~')
        self.data_dir = os.path.join(home_dir, 'agv', 'agv_data')
        
        try:
            os.makedirs(self.data_dir, exist_ok=True)
        except Exception as e:
            self.get_logger().error(f"Could not create directory {self.data_dir}: {e}")

        self.db_filename = 'map_data.db'
        self.db_path = os.path.join(self.data_dir, self.db_filename)
        self.db_connection = None
        
        # 2. Initialize Database
        self.connect_to_db()
        self.setup_database_schema()

        self.current_map = MarkerArray()
        self.marker_dictionary = ""
        self.marker_size = 0.0
        self.x_length = 0.0
        self.y_length = 0.0

        # Service Servers
        self.map_loader_server_ = self.create_service(LoadMap, '/map_database/load_map', self.loadMapCallback)
        self.map_remover_server_ = self.create_service(RemoveMap, '/map_database/remove_map', self.removeMapCallback)
        self.map_saver_server_ = self.create_service(SaveMap, '/map_database/save_map', self.handleSaveMap)
        self.map_getter_server_ = self.create_service(GetMap, '/map_database/get_map', self.getMapCallback)
        self.map_list_getter_server_ = self.create_service(GetMapName, '/map_database/get_map_name', self.getMapNameCallback)

        # Publisher
        self.map_signal_pub_ = self.create_publisher(String, '/map_database/map_update_signal', 10)

        self.get_logger().info(f'Map Server node has been started. DB Path: {self.db_path}')

    def connect_to_db(self):
        try:
            self.db_connection = sqlite3.connect(self.db_path, check_same_thread=False)
            self.db_connection.row_factory = sqlite3.Row
            # Enable Foreign Key support
            self.db_connection.execute("PRAGMA foreign_keys = ON;")
            self.get_logger().info('Successfully connected to SQLite database.')
            atexit.register(self.close_db_connection)
        except sqlite3.Error as e:
            self.get_logger().error(f"Failed to connect to database at {self.db_path}: {e}")

    def close_db_connection(self):
        if self.db_connection:
            self.db_connection.close()
            self.get_logger().info('Closed database connection.')

    def setup_database_schema(self):
        if not self.db_connection:
            return
        
        try:
            cursor = self.db_connection.cursor()
            # Maps Table
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS Maps (
                    map_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    map_name TEXT NOT NULL UNIQUE,
                    x_length FLOAT NOT NULL,
                    y_length FLOAT NOT NULL,
                    marker_dictionary TEXT NOT NULL,
                    marker_size FLOAT NOT NULL, 
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );
            """)
            # Markers Table - Added ON DELETE CASCADE for better cleanup
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS Markers (
                    marker_pk INTEGER PRIMARY KEY AUTOINCREMENT,
                    map_id INTEGER NOT NULL,
                    marker_id INTEGER NOT NULL,
                    x_coord FLOAT NOT NULL,
                    y_coord FLOAT NOT NULL,
                    orientation FLOAT NOT NULL,
                    FOREIGN KEY (map_id) REFERENCES Maps (map_id) ON DELETE CASCADE
                );
            """)
            self.db_connection.commit()
            self.get_logger().info("Database schema checked and ready.")
        except sqlite3.Error as e:
            self.get_logger().error(f"Schema setup failed: {e}")

    def handleSaveMap(self, request, response):
        response.success = False
        if not self.db_connection:
            response.message = "Database connection is unavailable."
            return response

        try:
            cursor = self.db_connection.cursor()
            # Start transaction explicitly
            cursor.execute("BEGIN TRANSACTION;")

            # 1. Handle Map Metadata
            cursor.execute("SELECT map_id FROM Maps WHERE map_name = ?", (request.map_name,))
            existing_record = cursor.fetchone()

            if existing_record:
                map_id = existing_record['map_id']
                cursor.execute("""UPDATE Maps SET x_length=?, y_length=?, marker_dictionary=?, 
                                  marker_size=?, created_at=CURRENT_TIMESTAMP WHERE map_id=?""", 
                               (request.x_length, request.y_length, request.marker_dictionary, request.marker_size, map_id))
            else:
                cursor.execute("""INSERT INTO Maps (map_name, x_length, y_length, marker_dictionary, marker_size) 
                                  VALUES (?, ?, ?, ?, ?)""", 
                               (request.map_name, request.x_length, request.y_length, request.marker_dictionary, request.marker_size))
                map_id = cursor.lastrowid

            # 2. Validate and Prepare Markers
            cursor.execute("DELETE FROM Markers WHERE map_id = ?", (map_id,))
            marker_data_list = []
            
            for marker in request.map_data.markers:
                quaternion = [marker.pose.pose.orientation.x, marker.pose.pose.orientation.y,
                              marker.pose.pose.orientation.z, marker.pose.pose.orientation.w]
                _, _, yaw = euler_from_quaternion(quaternion)
                
                marker_data_list.append((map_id, marker.id, marker.pose.pose.position.x, marker.pose.pose.position.y, yaw))

            if marker_data_list:
                cursor.executemany("INSERT INTO Markers (map_id, marker_id, x_coord, y_coord, orientation) VALUES (?, ?, ?, ?, ?)", marker_data_list)

            self.db_connection.commit()
            response.success = True
            response.new_map_id = map_id
            response.message = f"Map '{request.map_name}' saved. Markers: {len(marker_data_list)}."

        except Exception as e:
            if self.db_connection: self.db_connection.rollback()
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)

        return response

    def loadMapCallback(self, request, response):
        response.success = False
        if not self.db_connection:
            response.message = "Database connection unavailable."
            return response
        
        try:
            cursor = self.db_connection.cursor()
            cursor.execute("SELECT * FROM Maps WHERE map_name = ?", (request.map_name,))
            meta = cursor.fetchone()

            if not meta:
                response.message = f"Map '{request.map_name}' not found."
                return response
            
            cursor.execute("SELECT marker_id, x_coord, y_coord, orientation FROM Markers WHERE map_id = ?", (meta['map_id'],))
            marker_rows = cursor.fetchall()

            new_map_array = MarkerArray()
            for row in marker_rows:
                m = Marker()
                m.id = row['marker_id']
                m.pose.pose.position.x = row['x_coord']
                m.pose.pose.position.y = row['y_coord']
                q = quaternion_from_euler(0, 0, row['orientation'])
                m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w = q
                new_map_array.markers.append(m)

            # Update cache
            self.current_map = new_map_array
            self.marker_dictionary = meta['marker_dictionary']
            self.marker_size = meta['marker_size']
            self.x_length = meta['x_length']
            self.y_length = meta['y_length']

            self.map_signal_pub_.publish(String(data=f"MAP_UPDATE_SUCCESS: {request.map_name}"))
            response.success = True
            response.message = f"Loaded map '{request.map_name}'."

        except Exception as e:
            response.message = f"Load Error: {e}"
        return response

    def getMapCallback(self, request, response):
        if self.current_map.markers:
            response.map_data = self.current_map
            response.marker_dictionary = self.marker_dictionary
            response.marker_size = self.marker_size
            response.x_length = self.x_length
            response.y_length = self.y_length
            response.success = True
        else:
            response.success = False
            response.message = "No map loaded."
        return response
    
    def getMapNameCallback(self, request, response):
        try:
            cursor = self.db_connection.cursor()
            cursor.execute("SELECT map_name FROM Maps ORDER BY map_name ASC")
            response.map_names = [row['map_name'] for row in cursor.fetchall()]
            response.success = True
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    def removeMapCallback(self, request, response):
        try:
            cursor = self.db_connection.cursor()
            
            # 1. Get the map_id first
            cursor.execute("SELECT map_id FROM Maps WHERE map_name = ?", (request.map_name,))
            row = cursor.fetchone()
            
            if not row:
                response.success = False
                response.message = "Map not found."
                return response

            map_id = row['map_id']

            # 2. Manually delete children first (Safety First)
            cursor.execute("DELETE FROM Markers WHERE map_id = ?", (map_id,))
            
            # 3. Delete the parent
            cursor.execute("DELETE FROM Maps WHERE map_id = ?", (map_id,))

            self.db_connection.commit()
            response.success = True
            response.message = f"Map '{request.map_name}' and all associated markers deleted."
            
        except sqlite3.Error as e:
            self.db_connection.rollback()
            response.success = False
            response.message = f"Database error: {e}"
            
        return response
        
def main(args=None):
    rclpy.init(args=args)
    node = MapHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()