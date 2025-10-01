import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from aruco_msgs.msg import Marker, MarkerArray

class ArucoLocalization(Node):
    def __init__(self):
        super().__init__('aruco_localization')

        self.aruco_pose_sub_ = self.create_subscription(MarkerArray, '/aruco/pose', self.arucoPoseCallback, 10)

        self.get_logger().info('Aruco Localization node has been started.')

    def arucoPoseCallback(self, msg):
        self.get_logger().info(f'aruco_pos: {msg}')

def main(args=None):
    rclpy.init(args=args)
    aruco_localization = ArucoLocalization()
    rclpy.spin(aruco_localization)
    aruco_localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()