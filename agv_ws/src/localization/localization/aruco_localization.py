import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from aruco_msgs.msg import Marker, MarkerArray
from tf_transformations import compose_matrix, inverse_matrix, quaternion_matrix, concatenate_matrices, quaternion_from_matrix
import numpy as np

class ArucoLocalization(Node):
    def __init__(self):
        super().__init__('aruco_localization')

        self.current_map_data = None

        self.agv_cam_pose = [0.0, 0.0, 0.0] # x,y,z
        self.agv_cam_angle = [0.0, np.deg2rad(10), 0.0] # roll, pitch, yaw
        self.agv_cam_tf = compose_matrix(translate=self.agv_cam_pose, angles=self.agv_cam_angle)

        self.marker_world_pose = [2.0, 1.0, 0.0] # x,y,z
        self.marker_world_angle = [0.0, 0.0, 0.0] # roll, pitch, yaw
        self.marker_world_tf = compose_matrix(translate=self.marker_world_pose, angles=self.marker_world_angle)

        # Subscriptions
        self.aruco_pose_sub_ = self.create_subscription(MarkerArray, '/aruco/pose', self.arucoPoseCallback, 10)
        self.map_signal_sub_ = self.create_subscription(String, '/map_database/map_update_signal', self.mapSignalCallback, 10)

        # Service Client
        # self.map_loader_client_ = self.create_client(LoadMapService, '/map_database/get_map')

        # Publisher
        self.robot_pose_pub_  = self.create_publisher(PoseArray, '/robot/pose', 10)
        
        self.get_logger().info('Aruco Localization node has been started.')
        self.loadMap()

    def arucoPoseCallback(self, aruco_pos_msg):
        aruco_marker_array = aruco_pos_msg.markers

        robot_pose_array = PoseArray()
        robot_pose_array.header.stamp = self.get_clock().now().to_msg() 
        robot_pose_array.header.frame_id = 'world'

        for aruco_marker in aruco_marker_array:
            aruco_pose = aruco_marker.pose.pose
            
            position = aruco_pose.position
            orientation = aruco_pose.orientation

            marker_cam_tf = quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
            marker_cam_tf[:3, 3] = [position.x, position.y, position.z]

            agv_world_tf = concatenate_matrices(self.marker_world_tf, inverse_matrix(marker_cam_tf), self.agv_cam_tf)
            agv_world_pose = agv_world_tf[:3, 3] 
            agv_world_angle = quaternion_from_matrix(agv_world_tf)

            robot_pose = Pose()
            robot_pose.position.x = agv_world_pose[0]
            robot_pose.position.y = agv_world_pose[1]
            robot_pose.position.z = agv_world_pose[2]
            robot_pose.orientation.x = agv_world_angle[0]
            robot_pose.orientation.y = agv_world_angle[1]
            robot_pose.orientation.z = agv_world_angle[2]
            robot_pose.orientation.w = agv_world_angle[3]

            robot_pose_array.poses.append(robot_pose)

        self.robot_pose_pub_.publish(robot_pose_array)        

    def mapSignalCallback(self, map_signal_msg):
        self.loadMap()

    def loadMap(self):
        self.map_loader_req = None
        # self.map_loader_result = self.map_loader_client_.call_async(self.map_loader_req)

        # return self.map_loader_result.result()
        pass

def main(args=None):
    rclpy.init(args=args)
    aruco_localization = ArucoLocalization()
    rclpy.spin(aruco_localization)
    aruco_localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()