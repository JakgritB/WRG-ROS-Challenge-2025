#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class HumanDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('human_detection_subscriber')
        
        # Parameters
        self.declare_parameter('detection_topic', '/human_detections')
        self.declare_parameter('max_warning_level', 10)
        self.declare_parameter('warning_increment', 1)
        self.declare_parameter('warning_decrement', 2)
        
        # Get parameters
        detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        self.max_warning_level = self.get_parameter('max_warning_level').get_parameter_value().integer_value
        self.warning_increment = self.get_parameter('warning_increment').get_parameter_value().integer_value
        self.warning_decrement = self.get_parameter('warning_decrement').get_parameter_value().integer_value
        
        # State variables
        self.human_warning = 0
        self.last_detection_count = 0
        
        # Subscribers
        self.detection_subscription = self.create_subscription(
            Int32,
            detection_topic,
            self.detection_callback,
            10
        )
        
        # Publishers - สำหรับส่งคำสั่งควบคุมหุ่นยนต์ (ถ้าต้องการ)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Timer สำหรับ periodic processing
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        self.get_logger().info(f'Human Detection Subscriber initialized')
        self.get_logger().info(f'Subscribing to: {detection_topic}')
        self.get_logger().info(f'Max warning level: {self.max_warning_level}')
        
    def detection_callback(self, msg):
        """
        Callback สำหรับรับข้อมูลการตรวจจับมนุษย์
        """
        human_count = msg.data
        self.last_detection_count = human_count
        
        # Update warning level based on detection
        if human_count > 0:
            self.human_warning += self.warning_increment
            if self.human_warning > self.max_warning_level:
                self.human_warning = self.max_warning_level
        else:
            self.human_warning -= self.warning_decrement
            if self.human_warning < 0:
                self.human_warning = 0
        
        self.get_logger().info(
            f'Human Detection: Count={human_count}, Warning Level={self.human_warning}'
        )
        
        # ตรวจสอบระดับการเตือน และทำการตอบสนอง
        self.process_warning_level()
    
    def process_warning_level(self):
        """
        ประมวลผลระดับการเตือนและสั่งการหุ่นยนต์
        """
        if self.human_warning >= 8:  # ระดับเตือนสูง
            self.get_logger().warn('HIGH ALERT: Human detected! Stopping robot.')
            self.stop_robot()
        elif self.human_warning >= 5:  # ระดับเตือนปานกลาง
            self.get_logger().warn('MEDIUM ALERT: Human nearby. Reducing speed.')
            self.slow_robot()
        elif self.human_warning >= 2:  # ระดับเตือนต่ำ
            self.get_logger().info('LOW ALERT: Human detected in area.')
        else:
            self.get_logger().info('CLEAR: No humans detected.')
    
    def stop_robot(self):
        """
        หยุดหุ่นยนต์
        """
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
    
    def slow_robot(self):
        """
        ลดความเร็วหุ่นยนต์ (ตัวอย่าง - ปรับได้ตามต้องการ)
        """
        # สามารถเพิ่มโลจิกสำหรับลดความเร็วได้ที่นี่
        # หรือส่งสัญญาณไปยัง navigation stack
        pass
    
    def timer_callback(self):
        """
        Callback ที่ทำงานเป็นระยะๆ สำหรับการประมวลผลเพิ่มเติม
        """
        # สามารถเพิ่มโลจิกเพิ่มเติมได้ที่นี่
        # เช่น การตรวจสอบสถานะระบบ, การบันทึก log, etc.
        pass
    
    def get_warning_status(self):
        """
        ฟังก์ชันสำหรับให้โหนดอื่นเรียกใช้เพื่อทราบสถานะ
        """
        return {
            'warning_level': self.human_warning,
            'last_count': self.last_detection_count,
            'status': self.get_status_string()
        }
    
    def get_status_string(self):
        """
        แปลงระดับเตือนเป็นข้อความ
        """
        if self.human_warning >= 8:
            return "HIGH_ALERT"
        elif self.human_warning >= 5:
            return "MEDIUM_ALERT"
        elif self.human_warning >= 2:
            return "LOW_ALERT"
        else:
            return "CLEAR"

def main(args=None):
    rclpy.init(args=args)
    
    subscriber = HumanDetectionSubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        subscriber.get_logger().info('Shutting down human detection subscriber...')
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()