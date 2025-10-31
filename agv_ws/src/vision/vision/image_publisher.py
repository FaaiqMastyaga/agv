import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_device_path', rclpy.Parameter.Type.STRING),
                ('image_width', rclpy.Parameter.Type.INTEGER),
                ('image_height', rclpy.Parameter.Type.INTEGER),
                ('camera_name', rclpy.Parameter.Type.STRING),
                ('camera_model', rclpy.Parameter.Type.STRING),
                ('camera_matrix.data', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('distortion_coefficients.data', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('rectification_matrix.data', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('projection_matrix.data', rclpy.Parameter.Type.DOUBLE_ARRAY),
            ]
        )

        # Get parameter values
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.camera_model = self.get_parameter('camera_model').get_parameter_value().string_value
        self.camera_device_path = self.get_parameter('camera_device_path').get_parameter_value().string_value

        # Read the matrix from parameters
        self.camera_matrix = self.get_parameter('camera_matrix.data').get_parameter_value().double_array_value
        self.distortion_coefficients = self.get_parameter('distortion_coefficients.data').get_parameter_value().double_array_value
        self.rectification_matrix = self.get_parameter('rectification_matrix.data').get_parameter_value().double_array_value
        self.projection_matrix = self.get_parameter('projection_matrix.data').get_parameter_value().double_array_value

        # Create publisher for both Image and CameraInfo messages
        self.image_pub_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.camera_info_pub_ = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # Create a timer to publish data at a fixed rate
        timer_period = 0.1
        self.timer_ = self.create_timer(timer_period, self.timerCallback)
    
        # Initialize video capture
        self.cap = cv2.VideoCapture(self.camera_device_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video capture device at ID {self.camera_device_path}")
            self.destroy_node() # Or handle this gracefully
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)

        self.bridge = CvBridge()
        self.get_logger().info('Image Publisher node has been started.')
        self.get_logger().info(f'Loaded camera: {self.camera_device_path}, camera name: {self.camera_name}')
        self.get_logger().info(f'Camera Matrix: {self.camera_matrix}')

    def timerCallback(self):
        ret, frame = self.cap.read()
        if ret:
            # Create and publish the Image message
            img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_link'
            self.image_pub_.publish(img_msg)

            # Create and publish the CameraInfo message
            camera_info_msg = CameraInfo()
            camera_info_msg.header = img_msg.header
            camera_info_msg.height = self.image_height
            camera_info_msg.width = self.image_width
            camera_info_msg.distortion_model = self.camera_model
            camera_info_msg.d = self.distortion_coefficients
            camera_info_msg.k = self.camera_matrix
            camera_info_msg.r = self.rectification_matrix
            camera_info_msg.p = self.projection_matrix

            self.camera_info_pub_.publish(camera_info_msg)

            self.get_logger().info('Publishing image and camera info.')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()