import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from aruco_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf_transformations

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('video_width', 800)
        self.declare_parameter('video_height', 448)

        self.video_width = self.get_parameter('video_width').get_parameter_value().integer_value
        self.video_height = self.get_parameter('video_height').get_parameter_value().integer_value

        self.declare_parameter('aruco_dict', "DICT_4X4_50")
        # self.declare_parameter('marker_size', 4.0)
        self.declare_parameter('marker_size', 8.0)

        self.sub_ = self.create_subscription(Image, '/camera/image_raw', self.msgCallback, 10)
        self.aruco_pose_pub_ = self.create_publisher(MarkerArray, '/aruco/pose', 10)
        self.image_pub_ = self.create_publisher(Image, '/camera/image_marked', 10)

        self.bridge = CvBridge()

        desired_aruco_dict = self.get_parameter('aruco_dict').get_parameter_value().string_value
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

        self.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[desired_aruco_dict])
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        print(f"Marker size: {self.marker_size}")

        # Replace with your camera matrix
        self.camera_matrix = np.array([
            [585.72338132,  0.0,            409.22087225], 
            [0.0,           588.43905586,   231.17154493], 
            [0.0,           0.0,            1.0]
        ])
        # Replace with your distortion coefficients
        self.dist_coeffs = np.array([
            -0.01066598, -0.05213597, -0.00347335, -0.00135213,  0.04338231
        ])

    def msgCallback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)

        aruco_marker_array = MarkerArray()
        aruco_marker_array.header.frame_id = 'ArUco Pose'
        
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_parameters)

        if len(corners) > 0:
            ids = ids.flatten()

            for (marker_corner, marker_id) in zip(corners, ids):
                # Get marker pose (relative to camera)
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corner, self.marker_size, self.camera_matrix, self.dist_coeffs)
                rvec, tvec = rvec[0][0], tvec[0][0]

                # Extract information from marker pose
                position = tvec

                rotation_matrix = np.eye(4) 
                rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)
                quaternion = tf_transformations.quaternion_from_matrix(rotation_matrix)
                
                euler = tf_transformations.euler_from_quaternion(quaternion, 'szxy')
                euler = [np.rad2deg(angle) for angle in euler] 
                
                distance = np.linalg.norm(position)

                # Build message
                pose = Pose()
                pose.position.x = position[0]
                pose.position.y = position[1]
                pose.position.z = position[2]
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

                # Extract the marker corners
                corners = marker_corner.reshape((4,2))
                (top_left, top_right, bottom_right, bottom_left) = corners

                # Convert the (x,y) coordinate pairs to int
                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))

                # Draw the bounding box of the ArUco detection
                cv2.line(img, top_left, top_right, (0, 255, 0), 2)
                cv2.line(img, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(img, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(img, bottom_left, top_left, (0, 255, 0), 2)

                # Calculate and draw the center of the ArUco marker
                center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                center_y = int((top_left[1] + bottom_right[1]) / 2.0)
                center = (center_x, center_y)
                cv2.circle(img, center, 4, (0, 0, 255), -1)

                # Calculate and draw arrow from center of camera to center of ArUco marker
                center_cam_x = int(self.video_width / 2)
                center_cam_y = int(self.video_height / 2)
                center_cam = (center_cam_x, center_cam_y)
                cv2.arrowedLine(img, center_cam, center, (255, 255, 255), 2)
                cv2.line(img, center_cam, (center_x, center_cam_y), (0, 0, 255), 2)
                cv2.line(img, center_cam, (center_cam_x, center_y), (0, 255, 0), 2)

                # Draw the ArUco marker ID on the video frame
                # The ID is located at the top_left of the ArUco marker
                cv2.putText(img, str(marker_id), (top_left[0], top_right[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Print ID and distance
                self.get_logger().info(f'ID: {marker_id}\n\tposition: {position}\n\teuler: {euler}\n\tdistance: {distance}')
        
        cv2.imshow('Video frame', img)
        cv2.waitKey(1)
        
        img_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        self.image_pub_.publish(img_msg)

        self.aruco_pose_pub_.publish(aruco_marker_array)

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()