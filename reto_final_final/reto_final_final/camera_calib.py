import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os

class ImageProcessor(Node):
    def __init__(self):

        super().__init__('image_processor')

        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(
            Image,
            '/processed',
            10)
        
        self.bridge = CvBridge()

        self.get_logger().info('ImageProcessor node started.')

    def listener_callback(self, msg):

        # Convert ROS Image to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Scale to half resolution
        height, width = cv_image.shape[:2]
        scaled = cv2.resize(cv_image, (width // 2, height // 2), interpolation=cv2.INTER_AREA)
        # Crop to lower half only
        h = scaled.shape[0]
        cropped = scaled[h // 2 : h, :]

        # Publish processed image
        processed_msg = self.bridge.cv2_to_imgmsg(cropped, encoding='bgr8')
        self.publisher_.publish(processed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()