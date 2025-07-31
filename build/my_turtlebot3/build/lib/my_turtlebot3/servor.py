#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool  # สำหรับส่งค่า Boolean
from gpiozero import AngularServo
from time import sleep
import signal
import sys

class Servo_Control_Node(Node):
    def __init__(self):
        super().__init__('servo_control_node')  # create node name

        # สร้างเซอร์โว
        self.servo_set = AngularServo(17, min_pulse_width=0.0006, max_pulse_width=0.0023)
        self.servo_shoot = AngularServo(27, min_pulse_width=0.0006, max_pulse_width=0.0023)

        # ตั้งค่ามุม
        self.set_pose_cube = -3
        self.set_wait_cube = -6
        self.set_open_cube = 60
        self.set_pose = -90
        self.set_wait = -50
        self.set_shoot = 0

        # ตั้งค่าเริ่มต้น
        self.servo_set.angle = self.set_wait_cube
        self.servo_shoot.angle = self.set_wait
        sleep(0.5)

        # subscribe image from topic: 'person_detected'
        self.subscription = self.create_subscription(
            Bool,
            'person_detected',
            self.status_callback,
            10
        )

        self.is_servo_running = False  # ตัวแปรเก็บสถานะของเซอร์โว

    def servo_start(self):
        # เซอร์โวเริ่มทำงาน
        self.servo_set.angle = self.set_open_cube
        sleep(0.5)
        self.servo_shoot.angle = self.set_shoot
        sleep(0.5)
        self.servo_shoot.angle = self.set_pose
        sleep(1)
        self.servo_shoot.angle = self.set_wait
        sleep(0.5)
        self.servo_set.angle = self.set_wait_cube
        sleep(0.5)

    def status_callback(self, msg):
        try:
            if msg.data and not self.is_servo_running:  # ตรวจสอบว่าเซอร์โวยังไม่ได้ทำงาน
                print("Camera Detected -> Servo Control Start")
                self.is_servo_running = True  # ตั้งค่าสถานะให้เซอร์โวทำงาน
                self.servo_start()
            elif not msg.data:
                print("Camera Wait for Detect -> Servo Control Stop")
                self.is_servo_running = False  # ตั้งค่าสถานะเป็น False เมื่อไม่มีการตรวจจับ
        except Exception as e:
            self.get_logger().error('Error : %s' % str(e))

    def shutdown(self):
        # ปิดการทำงานอย่างถูกต้อง
        print("Shutting down the node...")
        self.servo_set.angle = self.set_wait_cube
        self.servo_shoot.angle = self.set_wait
        sleep(0.5)  # ให้เวลาปิดการทำงานเซอร์โว
        self.get_logger().info("Node shutdown complete.")

def main(args=None):
    rclpy.init(args=args)
    node = Servo_Control_Node()

    # จับสัญญาณ Ctrl+C
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()  # เมื่อกด Ctrl+C, ปิดการทำงาน

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()