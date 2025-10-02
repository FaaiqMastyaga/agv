import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from aruco_msgs.msg import Marker, MarkerArray

class ArucoLocalization(Node):
    def __init__(self):
        super().__init__('aruco_localization')

        self.current_map_data = None

        # Subscriptions
        self.aruco_pose_sub_ = self.create_subscription(MarkerArray, '/aruco/pose', self.arucoPoseCallback, 10)
        self.map_signal_sub_ = self.create_subscription(String, '/map_database/map_update_signal', self.mapSignalCallback, 10)

        # Service Client
        self.map_loader_client_ = self.create_client(LoadMapService, '/map_database/get_map')

        # Publisher
        self.robot_pose_pub_  = self.create_publisher(Pose2D, '/robot/pose', 10)
        
        self.get_logger().info('Aruco Localization node has been started.')
        self.loadMap()

    def arucoPoseCallback(self, aruco_pos_msg):
        # process data distance aruco (tranform normal distance to horizontal distance)
        # process horizontal distance and map data into agv position
        pass

    def mapSignalCallback(self, map_signal_msg):
        self.loadMap()

    def loadMap(self):
        self.map_loader_req = None
        self.map_loader_result = self.map_loader_client_.call_async(self.map_loader_req)

        return self.map_loader_result.result()

def main(args=None):
    rclpy.init(args=args)
    aruco_localization = ArucoLocalization()
    rclpy.spin(aruco_localization)
    aruco_localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()