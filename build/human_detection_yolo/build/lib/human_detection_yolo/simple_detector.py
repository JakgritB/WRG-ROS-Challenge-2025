#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2

class SimpleDetector(Node):
    def __init__(self):
        super().__init__('simple_detector')
        
        self.bridge = CvBridge()
        
        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(Int32, '/human_detections', 10)
        self.image_pub = self.create_publisher(Image, '/camera/annotated_image', 10)
        
        self.get_logger().info('Simple Detector started')
        
    def image_callback(self, msg):
        try:
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Simple processing - just add text and publish
            annotated = cv_image.copy()
            cv2.putText(annotated, 'Camera Working!', (50, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            annotated_msg.header = msg.header
            self.image_pub.publish(annotated_msg)
            
            # Publish dummy detection
            detection_msg = Int32()
            detection_msg.data = 1  # Dummy value
            self.detection_pub.publish(detection_msg)
            
            self.get_logger().info('Image processed and published')
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main():
    rclpy.init()
    node = SimpleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()