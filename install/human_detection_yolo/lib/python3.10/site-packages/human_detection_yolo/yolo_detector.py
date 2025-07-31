#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Header, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

# สร้าง custom message สำหรับส่งข้อมูลการตรวจจับ
from std_msgs.msg import Float32MultiArray

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detection_topic', '/human_detections')
        
        # Get parameters
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        
        self.get_logger().info(f'Initializing YOLO Detector...')
        self.get_logger().info(f'Model path: {model_path}')
        self.get_logger().info(f'Image topic: {image_topic}')
        self.get_logger().info(f'Detection topic: {detection_topic}')
        
        # Initialize YOLO model
        try:
            self.get_logger().info('Loading YOLO model...')
            self.model = YOLO(model_path)
            self.get_logger().info(f'YOLO model loaded successfully: {model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self.get_logger().error('Make sure ultralytics is installed: pip3 install ultralytics')
            raise e
        
        # CV Bridge
        try:
            self.bridge = CvBridge()
            self.get_logger().info('CV Bridge initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CV Bridge: {e}')
            raise e
        
        # Subscribers and Publishers
        self.get_logger().info('Creating subscribers and publishers...')
        
        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f'Image subscriber created for topic: {image_topic}')
        
        # Publisher สำหรับจำนวนคนที่ตรวจพบ
        self.detection_publisher = self.create_publisher(
            Int32,
            detection_topic,
            10
        )
        self.get_logger().info(f'Detection publisher created for topic: {detection_topic}')
        
        # Publisher สำหรับภาพที่มี bounding box
        self.annotated_image_publisher = self.create_publisher(
            Image,
            '/camera/annotated_image',
            10
        )
        self.get_logger().info('Annotated image publisher created for topic: /camera/annotated_image')
        
        self.get_logger().info(f'YOLO Detector initialized')
        self.get_logger().info(f'Subscribing to: {image_topic}')
        self.get_logger().info(f'Publishing detections to: {detection_topic}')
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Run YOLO inference
            results = self.model(cv_image, conf=self.confidence_threshold)
            
            # Count humans (class 0 in COCO dataset is 'person')
            human_count = 0
            annotated_image = cv_image.copy()
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Check if detected class is person (class 0)
                        if int(box.cls[0]) == 0:  # Person class
                            human_count += 1
                            
                            # Draw bounding box
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            confidence = float(box.conf[0])
                            
                            # Draw rectangle and label
                            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.putText(annotated_image, f'Person: {confidence:.2f}', 
                                      (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Publish human count
            count_msg = Int32()
            count_msg.data = human_count
            self.detection_publisher.publish(count_msg)
            
            # Publish annotated image
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, 'bgr8')
                annotated_msg.header = msg.header
                self.annotated_image_publisher.publish(annotated_msg)
            except Exception as e:
                self.get_logger().warn(f'Failed to publish annotated image: {e}')
            
            self.get_logger().info(f'Detected {human_count} humans')
            
        except Exception as e:
            self.get_logger().error(f'Error in image processing: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    detector = YOLODetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        detector.get_logger().info('Shutting down YOLO detector...')
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()