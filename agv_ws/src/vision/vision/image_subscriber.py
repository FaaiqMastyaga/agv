import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.sub_ = self.create_subscription(Image, '/camera/image_raw', self.msgCallback, 10)
        
        self.bridge = CvBridge()

    def msgCallback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow('Video frame', img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()