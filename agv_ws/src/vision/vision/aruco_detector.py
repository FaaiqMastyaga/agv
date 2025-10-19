import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from aruco_msgs.msg import Marker, MarkerArray
from map_database_interfaces.srv import GetMap
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf_transformations
import message_filters
import threading

class ArucoDetector(Node):
    ARUCO_DICT = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
        "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
        "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
        "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
        "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
    }

    def __init__(self):
        super().__init__('aruco_detector')

        self.config_lock = threading.Lock()

        self.aruco_dict_name = 'DICT_4X4_50'
        self.marker_size = 8.0 # cm

        self.aruco_dict = cv2.aruco.Dictionary_get(self.ARUCO_DICT[self.aruco_dict_name])
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

        self.camera_matrix = None
        self.distortion_coeffs = None
        self.camera_info_received = False
        self.get_logger().info('ArUco Detector node has been started. Waiting for camera info...')

        # Subscriptions
        image_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        camera_info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/camera_info')
        self.map_signal_sub_ = self.create_subscription(String, '/map_database/map_update_signal', self.mapSignalCallback, 10)

        # Service Client
        self.map_loader_client_ = self.create_client(GetMap, '/map_database/get_map')

        # Time synchronizer
        self.ts = message_filters.TimeSynchronizer([image_sub, camera_info_sub], 10)
        self.ts.registerCallback(self.imageInfoCallback)

        # Publishers
        self.aruco_pose_pub_ = self.create_publisher(MarkerArray, '/aruco/pose', 10)
        self.image_marked_pub_ = self.create_publisher(Image, '/camera/image_marked', 10)

    def imageInfoCallback(self, image_msg, camera_info_msg):
        with self.config_lock:
            # Update camera parameters from the CameraInfo message
            self.camera_matrix = np.array(camera_info_msg.k).reshape(3,3)
            self.distortion_coeffs = np.array(camera_info_msg.d)

            current_dict = self.aruco_dict
            current_size = self.marker_size
            current_cam_matrix = self.camera_matrix
            current_dist_coeffs = self.distortion_coeffs

        # Process the image
        img = self.bridge.imgmsg_to_cv2(image_msg)
        aruco_marker_array = MarkerArray()
        aruco_marker_array.header = image_msg.header

        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, current_dict, parameters=self.aruco_parameters)

        if len(corners) > 0:
            ids = ids.flatten()

            for (marker_corner, marker_id) in zip(corners, ids):
                # Get marker pose (relative to camera)
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corner, current_size, current_cam_matrix, current_dist_coeffs)
                rvec, tvec = rvec[0][0], tvec[0][0]

                # Convert rotation vector to quaternion
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                homogenous_matrix = np.eye(4)
                homogenous_matrix[:3, :3] = rotation_matrix
                quaternion = tf_transformations.quaternion_from_matrix(homogenous_matrix)

                # Build message
                pose = Pose()
                pose.position.x = tvec[0]
                pose.position.y = tvec[1]
                pose.position.z = tvec[2]
                pose.orientation.x = quaternion[0]
                pose.orientation.y = quaternion[1]
                pose.orientation.z = quaternion[2]
                pose.orientation.w = quaternion[3]

                aruco_marker = Marker()
                aruco_marker.id = int(marker_id)
                aruco_marker.pose.pose = pose
                aruco_marker.pose.covariance = (0.1 * np.eye(6)).flatten().tolist()
                aruco_marker.confidence = 0.1

                aruco_marker_array.markers.append(aruco_marker)

                top_left_corner = tuple(marker_corner[0][0].astype(int))

                # Draw bounding box and info (for visualization)
                cv2.aruco.drawDetectedMarkers(img, corners)
                cv2.drawFrameAxes(img, self.camera_matrix, self.distortion_coeffs, rvec, tvec, 3.0)
                cv2.putText(img, f"id: {marker_id}", (top_left_corner[0], top_left_corner[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Publish results
        self.aruco_pose_pub_.publish(aruco_marker_array)
        img_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        img_msg.header = image_msg.header
        self.image_marked_pub_.publish(img_msg)

    def mapSignalCallback(self, map_signal_msg):
        self.loadMap()

    def loadMap(self):
        self.map_loader_req = GetMap.Request()
        future = self.map_loader_client_.call_async(self.map_loader_req)
        future.add_done_callback(self.updateMapParam)

    def updateMapParam(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        
        if result.success:
            new_dict_name = result.marker_dictionary
            new_size = result.marker_size

            with self.config_lock:
                self.aruco_dict_name = new_dict_name
                self.marker_size = new_size
                self.aruco_dict = cv2.aruco.Dictionary_get(self.ARUCO_DICT[self.aruco_dict_name])
                self.get_logger().info(f'Loaded map with dictionary: {self.aruco_dict_name}, size: {self.marker_size} cm.')
        else:
            self.get_logger().error('Map service reported failure.')

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()