#!/usr/bin/env python3

from std_msgs.msg import Empty
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectionNode(Node):
    def __init__(self):
        self.node = rclpy.create_node('detection_node')
        
        # Initialize CV Bridge
        try:
            self.bridge = CvBridge()
            self.node.get_logger().info('✅ CV Bridge loaded successfully!')
        except Exception as e:
            self.node.get_logger().error(f'❌ CV Bridge failed: {e}')
            return
        
        # Load YOLO model
        try:
            from ultralytics import YOLO
            self.node.get_logger().info('📥 Loading YOLO model...')
            self.model = YOLO('yolov8n.pt')  # Will download automatically first time
            self.node.get_logger().info('✅ YOLO model loaded successfully!')
        except Exception as e:
            self.node.get_logger().error(f'❌ YOLO loading failed: {e}')
            self.model = None
        
        # รอรับสัญญาณจาก Node 1 ว่าไปถึง waypoint แล้ว
        self.subscriber = self.node.create_subscription(Empty, '/waypoint_reached', self.callback, 20)
        # Publisher สำหรับแจ้งว่าพบคน
        self.publisher = self.node.create_publisher(Empty, '/person_detect', 20)

        # Publisher สำหรับแจ้งว่าไม่พบคน (ถ้าจำเป็น)
        self.publisher_nav = self.node.create_publisher(Empty, '/person_no_detect', 20)

        
        # Subscribe to camera for YOLO detection
        self.image_subscription = self.node.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,10
        )
        
        # Optional: Publisher for annotated images (for debugging)
        self.annotated_image_publisher = self.node.create_publisher(Image, '/detection/annotated_image', 20)
        
        self.person_detected = False
        self.latest_detection_result = False
        self.detection_active = False
        
        self.node.get_logger().info('🚀 Detection Node ready!')
    
    def callback(self, msg):
        # เริ่มทำการตรวจจับคนเมื่อได้รับสัญญาณจาก Node 1
        print("Waypoint reached, starting person detection...")
        self.detection_active = True
        
        # ใช้ผลลัพธ์ล่าสุดจาก YOLO detection
        self.person_detected = self.latest_detection_result

        if not self.person_detected:
            self.publisher_nav.publish(Empty())
        
        else:
            self.publisher.publish(Empty())

        print("Published person detection status:", self.person_detected)
        
        # หยุดการตรวจจับหลังจากส่งผลลัพธ์แล้ว (optional)
        # self.detection_active = False
    
    def image_callback(self, msg):
        """Process camera images with YOLO detection continuously"""
        if self.model is None:
            return
            
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                self.node.get_logger().error('❌ Failed to decode compressed image')
                return
            
            # Run YOLO detection
            results = self.model(cv_image, conf=0.5, verbose=False)
            
            # Count humans and create annotated image
            human_count = 0
            annotated_image = cv_image.copy()
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Check if detected object is a person (class 0 in COCO dataset)
                        if int(box.cls[0]) == 0:  # Person class
                            human_count += 1
                            
                            # Get bounding box coordinates
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            confidence = float(box.conf[0])
                            
                            # Draw bounding box
                            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            
                            # Add confidence label
                            label = f'Person: {confidence:.2f}'
                            cv2.putText(annotated_image, label, (x1, y1-10), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Update detection result
            self.latest_detection_result = human_count > 0
            
            # Add detection status to image
            status_text = f'Persons: {human_count}'
            cv2.putText(annotated_image, status_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            detection_status = "ACTIVE" if self.detection_active else "STANDBY"
            cv2.putText(annotated_image, f'Status: {detection_status}', (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # Publish annotated image (optional, for debugging)
            try:
                ros_image = self.bridge.cv2_to_imgmsg(annotated_image, 'bgr8')
                ros_image.header.stamp = msg.header.stamp
                ros_image.header.frame_id = "camera"
                self.annotated_image_publisher.publish(ros_image)
            except Exception as e:
                self.node.get_logger().error(f'❌ Failed to publish annotated image: {e}')
            
            # Log detection results (only when detection is active)
            if self.detection_active and human_count > 0:
                self.node.get_logger().info(f'👥 Detected {human_count} person(s)')
                


        except Exception as e:
            self.node.get_logger().error(f'❌ Error processing image: {e}')
    
    def detect_person(self):
        # เก็บไว้เป็น fallback method (ใช้ผลลัพธ์จาก YOLO แทน)
        return self.latest_detection_result
        
    def spin(self):
        rclpy.spin(self.node)


def main(args=None):
    rclpy.init(args=args)
    
    # Run the detection node
    detection_node = DetectionNode()
    
    try:
        detection_node.spin()
    except KeyboardInterrupt:
        detection_node.node.get_logger().info('👋 Shutting down Detection Node...')
    finally:
        detection_node.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()