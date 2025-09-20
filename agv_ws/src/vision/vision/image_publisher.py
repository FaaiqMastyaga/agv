import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        # video_width: 1280
        # video_height: 720
        self.declare_parameter('video_width', 800)
        self.declare_parameter('video_height', 448)

        video_width = self.get_parameter('video_width').get_parameter_value().integer_value
        video_height = self.get_parameter('video_height').get_parameter_value().integer_value

        self.pub_ = self.create_publisher(Image, '/camera/image_raw', 10)
        timer_period = 0.1
        self.timer_ = self.create_timer(timer_period, self.timerCallback)
    
        self.cap = cv2.VideoCapture(3)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, video_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, video_height)

        self.bridge = CvBridge()

    def timerCallback(self):
        ret, frame = self.cap.read()
        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.pub_.publish(img_msg)
            # self.get_logger().info('Publishing video frame')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()