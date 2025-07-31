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
        self.camera_sub = self.create_subscription(Empty, '/person_no_detected', self.wait_for_camera, 20)
    
        self.goal_pose = [
        [0.8059219121932983, -0.17383164167404175, 0.0, 0.0, 0.0, 0.004246642976211978, 0.9999909829710629], #point1 drop 0
        [2.5960423946380615, -0.15482759475708008, 0.0, 0.0, 0.0,-0.7070412982237968, 0.707172258085686], #point2 ways 1

        [2.455105781555176, -1.4594608640670776, 0.0, 0.0, 0.0, -0.7085511291426619, 0.705659476936758], # point 3 drop 2
        [2.4662322998046875, -2.0100863952636719, 0.0, 0.0, 0.0, -0.7128670455634724, 0.7012992052965744],  # point 4 drop 3
        
        [2.4487218856811523, -2.2100892066955566, 0.0, 0.0, 0.0, -0.9999983781670552, 0.0018010172845805761 ], # point 5 ways 4
        [1.0779128074645996, -1.9279274940490723, 0.0, 0.0, 0.0, -0.9014558827851237,  0.43287098700662924],  # point 6 drop 5
        
                
        [0.40123450756073, -1.932122826576233, 0.0, 0.0, 0.0, 0.912893420940346,0.40819799363033654], # point 7 drop 6
        [0.43536388874053955,-1.9075031280517578, 0.0, 0.0, 0.0, 0.9089338852665702, 0.4169402741571237],  # point 8 ways 7
        
        
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

            if i in [0,2,3,5,6]:
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



