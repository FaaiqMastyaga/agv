import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('aruco_dict', "DICT_4X4_50")
        # self.declare_parameter('marker_size', 4.0)
        self.declare_parameter('marker_size', 8.0)

        self.sub_ = self.create_subscription(Image, '/camera/image_raw', self.msgCallback, 10)
        self.pub_ = self.create_publisher(PoseArray, 'aruco_pose', 10)
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

        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_parameters)

        if len(corners) > 0:
            ids = ids.flatten()

            for (marker_corner, marker_id) in zip(corners, ids):
                # Get marker pose (relative to camera)
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corner, self.marker_size, self.camera_matrix, self.dist_coeffs)
                distance = np.linalg.norm(tvec[0][0])

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
                cv2.circle(img, (center_x, center_y), 4, (0, 0, 255), -1)

                # Draw the ArUco marker ID on the video frame
                # The ID is located at the top_left of the ArUco marker
                cv2.putText(img, str(marker_id), (top_left[0], top_right[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Print ID and coordinate
                # self.get_logger().info(f'ID: {marker_id}, coordinate: {corners}')

                # Print ID and distance
                self.get_logger().info(f'ID: {marker_id}\n\trvec: {rvec}\n\ttvec: {tvec}\n\tdistance: {distance}')
        
        img_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        self.image_pub_.publish(img_msg)
        cv2.imshow('Video frame', img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()