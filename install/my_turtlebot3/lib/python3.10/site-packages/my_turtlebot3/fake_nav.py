#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import signal
import sys
import threading
import time

class FakeServoSubscriber(Node):
    def __init__(self):
        super().__init__('fake_servo_subscriber')
        
        self.get_logger().info('🤖 Fake Servo Subscriber Starting...')
        
        # Subscribe to servo_completed topic
        self.subscription = self.create_subscription(
            Empty,
            '/servo_completed',
            self.servo_completed_callback,
            10
        )
        
        # Status tracking
        self.last_received_time = None
        self.timeout_duration = 2.0  # seconds
        
        # Start monitoring thread
        self.monitoring_thread = threading.Thread(target=self.monitor_status, daemon=True)
        self.monitoring_thread.start()
        
        self.get_logger().info('🎯 Listening to /servo_completed topic')
        self.get_logger().info('⏰ Will print 0 if no message received for 2 seconds')
    
    def servo_completed_callback(self, msg):
        """Callback when servo_completed message is received"""
        try:
            print("1")  # Print 1 when message received
            self.last_received_time = time.time()
            self.get_logger().info('✅ Received servo_completed signal')
        except Exception as e:
            self.get_logger().error(f'❌ Callback error: {e}')
    
    def monitor_status(self):
        """Monitor thread to print 0 when no messages received"""
        while True:
            try:
                current_time = time.time()
                
                # If we haven't received any message yet, or timeout exceeded
                if (self.last_received_time is None or 
                    current_time - self.last_received_time > self.timeout_duration):
                    print("0")  # Print 0 when no message
                
                time.sleep(1.0)  # Check every second
                
            except Exception as e:
                self.get_logger().error(f'❌ Monitor error: {e}')
                time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    node = FakeServoSubscriber()
    
    def signal_handler(sig, frame):
        print('\n⏹️ Shutting down fake subscriber...')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        print("🤖 Fake Servo Subscriber ready!")
        print("📋 Output: 1 = servo completed, 0 = no signal")
        print("=" * 50)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()