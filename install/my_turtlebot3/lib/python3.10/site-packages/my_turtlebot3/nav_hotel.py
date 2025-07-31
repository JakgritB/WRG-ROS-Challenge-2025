#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
import tf_transformations
import time

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigator_node')

        # --- Init navigator     
        self.navigator = BasicNavigator()

        self.servo_done = False   #status servo
        self.camera_done = False  #status camera
        self.waypoints = 0          # number point robot pass waypoint

        # --- Set initial pose
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)  
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.x = q_x
        initial_pose.pose.orientation.y = q_y
        initial_pose.pose.orientation.z = q_z
        initial_pose.pose.orientation.w = q_w
        self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()

        # --- Publisher/Subscriber
        self.waypoint_reached_pub = self.create_publisher(Empty, '/waypoint_reached', 20)
        self.end_process_pub = self.create_publisher(Empty, '/task_completed', 20)
        self.servo_sub = self.create_subscription(Empty, '/servo_completed', self.wait_for_servo, 20)
        self.camera_sub = self.create_subscription(Empty, '/person_no_detect', self.wait_for_camera, 20)
    
        self.goal_pose = [
        [1.0823944807052612, 0.3457954525947571, 0.0, 0.0, 0.0, -0.013137852630418315, 0.9999136946898274], #point1 drop 0
        [1.4078201055526733, 0.28111886978149414, 0.0, 0.0, 0.0, -0.002501680823332833, 0.9999968707916331], #point2 ways 1
        [0.9586144685745239, 0.8087682723999023, 0.0, 0.0, 0.0, 0.9035204364637003,  0.4285450045122968],  # point 4 drop 3
        
        [0.2648913860321045, 0.7702072858810425, 0.0, 0.0, 0.0, 0.8624189632730107, 0.5061951518802855], # point 5 ways 4   
        
        [0.0,        0.0,       0.0, 0.0, 0.0, 0.0, 0.0]  # กลับจุดเริ่ม
        ]

        self.start_navigation()

    def wait_for_servo(self, msg):
        self.get_logger().info("Servo completed signal received.")
        self.servo_done = True

    def wait_for_camera(self, msg):
        self.get_logger().info("Camera (no person) signal received.")
        self.camera_done = True

    def wait_for_signal(self):
        # รอจนกว่า flag ใด flag หนึ่งจะเป็น True
        while not (self.servo_done or self.camera_done):
            rclpy.spin_once(self, timeout_sec=0.1)
        # รีเซ็ต flag สำหรับรอบถัดไป
        self.servo_done = False
        self.camera_done = False

    def start_navigation(self):
        for i, pose in enumerate(self.goal_pose):
            q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(pose[3], pose[4], pose[5])
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = pose[0]
            goal_pose.pose.position.y = pose[1]
            goal_pose.pose.position.z = pose[2]
            goal_pose.pose.orientation.x = q_x
            goal_pose.pose.orientation.y = q_y
            goal_pose.pose.orientation.z = q_z
            goal_pose.pose.orientation.w = q_w

            self.get_logger().info(f"Navigating to waypoint {i}")
            self.navigator.goToPose(goal_pose)

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                # self.get_logger().info(str(feedback))  # Optional

            if i in [0,1,2,3]:
                time.sleep(3.0)  # <- หยุดรอจริงๆ 3 วินาที

                self.waypoint_reached_pub.publish(Empty())
                self.waypoints+=1
                self.get_logger().info(f"Reached waypoint {self.waypoints}, waiting for servo or camera")
                self.wait_for_signal()
                

        self.end_process_pub.publish(Empty())
        self.get_logger().info("All waypoints completed. Task finished.")

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



