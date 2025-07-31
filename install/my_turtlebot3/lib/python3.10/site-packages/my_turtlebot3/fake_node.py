#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class FakePersonPublisher(Node):
    def __init__(self):
        super().__init__('fake_person_publisher')
        self.publisher_ = self.create_publisher(Empty, '/person_detect', 10)
        self.timer = self.create_timer(1.0, self.publish_message)  # ส่งทุก 1 วินาที

    def publish_message(self):
        msg = Empty()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Empty message to /person_detect')

def main(args=None):
    rclpy.init(args=args)
    node = FakePersonPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down fake publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
