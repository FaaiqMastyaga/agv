import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from aruco_msgs.msg import Marker, MarkerArray
from map_database_interfaces.srv import GetMap
from tf_transformations import compose_matrix, inverse_matrix, quaternion_matrix, concatenate_matrices, quaternion_from_matrix
import numpy as np

class ArucoLocalization(Node):
    def __init__(self):
        super().__init__('aruco_localization')

        self.marker_world_tf_array = []

        # Subscriptions
        self.aruco_pose_sub_ = self.create_subscription(MarkerArray, '/aruco/pose', self.arucoPoseCallback, 10)
        self.map_signal_sub_ = self.create_subscription(String, '/map_database/map_update_signal', self.mapSignalCallback, 10)

        # Service Client
        self.map_loader_client_ = self.create_client(GetMap, '/map_database/get_map')

        # Publisher
        self.agv_pose_hypothesis_pub_  = self.create_publisher(PoseArray, '/agv/pose_hypothesis', 10)
        self.agv_pose_pub_  = self.create_publisher(Pose, '/agv/pose', 10)
        
        self.get_logger().info('Aruco Localization node has been started.')
        self.loadMap()

    def arucoPoseCallback(self, aruco_pos_msg):
        aruco_marker_array = aruco_pos_msg.markers

        agv_pose_array = PoseArray()
        agv_pose_array.header.stamp = self.get_clock().now().to_msg() 
        agv_pose_array.header.frame_id = 'world'

        for aruco_marker in aruco_marker_array:
            aruco_pose = aruco_marker.pose.pose
            
            marker_id = aruco_marker.id
            position = aruco_pose.position
            orientation = aruco_pose.orientation

            marker_cam_tf = quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
            marker_cam_tf[:3, 3] = [position.x, position.y, position.z]

            for marker in self.marker_world_tf_array:
                if marker['id'] == marker_id:
                    agv_world_tf = concatenate_matrices(marker['marker_world_tf'], inverse_matrix(marker_cam_tf))
                    agv_world_pose = agv_world_tf[:3, 3] 
                    agv_world_angle = quaternion_from_matrix(agv_world_tf)
                    
                    agv_pose = Pose()
                    agv_pose.position.x = agv_world_pose[0]
                    agv_pose.position.y = agv_world_pose[1]
                    agv_pose.position.z = agv_world_pose[2]
                    agv_pose.orientation.x = agv_world_angle[0]
                    agv_pose.orientation.y = agv_world_angle[1]
                    agv_pose.orientation.z = agv_world_angle[2]
                    agv_pose.orientation.w = agv_world_angle[3]

                    agv_pose_array.poses.append(agv_pose)
        
                    break

        self.agv_pose_hypothesis_pub_.publish(agv_pose_array)        

    def mapSignalCallback(self, map_signal_msg):
        self.loadMap()

    def loadMap(self):
        self.map_loader_req = GetMap.Request()
        future = self.map_loader_client_.call_async(self.map_loader_req)
        future.add_done_callback(self.mapResponseCallback)

    def mapResponseCallback(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

        if result.success:
            map_data = result.map_data
            self.marker_world_tf_array = []

            for aruco_marker in map_data.markers:
                marker_id = aruco_marker.id
                aruco_pose = aruco_marker.pose.pose

                position = aruco_pose.position
                orientation = aruco_pose.orientation

                marker_world_tf = quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
                marker_world_tf[:3, 3] = [position.x, position.y, position.z]

                self.marker_world_tf_array.append(
                    {'id': marker_id, 'marker_world_tf': marker_world_tf}
                )
            
            self.get_logger().info(f'Loaded {len(self.marker_world_tf_array)} markers from map.')
        else:
            self.get_logger().error('Map service reported failure.')

def main(args=None):
    rclpy.init(args=args)
    aruco_localization = ArucoLocalization()
    rclpy.spin(aruco_localization)
    aruco_localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()