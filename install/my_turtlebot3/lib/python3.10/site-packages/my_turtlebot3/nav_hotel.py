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

        # self.goal_pose = [
        #     [0.7910786271095276, 0.0712955892086029, 0.2, 0.0, 0.0, 0.0, 0.0],
        #     [2.644859790802002, -0.4215397834777832, 0.2, 0.0, 0.0, 0.0, -1.57],
        #     [2.6608970165252686, -1.3731415271759033, 0.2, 0.0, 0.0, 0.000, -1.57],
        #     [2.6590499877929688, -1.7423019409179688, 0.2, 0.0, 0.0, 0.000, -1.57],
        #     [1.0085116624832153, -2.196805715560913, 0.2, 0.0, 0.0, 0.000, -3.14],
        #     [0.6081547737121582, -2.1948025226593018, 0.2, 0.0, 0.0, 0.000, -3.14],
        #     [0.0006335973739624023, 0.049252718687057495, 0.2, 0.0, 0.0, 0.0, 1] 
        # ]

    
        self.goal_pose = [
            [0.8319569230079651, 0.007189632393419743, 0.2, 0.0, 0.0, 0.0, 0.0],    # Drop  0
            [1.7417068481445312, -0.5569775390625, 0.2, 0.0, 0.0, 0.0, 1.57],       # Drop  1 เอาลงมา    เดิม y    =   -0.5269775390625
            [1.7067776918411255, -0.9723507165908813, 0.2, 0.0, 0.0, 0.0, 0.0],     # path  2
            [2.5295596313476562, -1.466878890991211, 0.2, 0.0, 0.0, 0.0, -1.57],    # Drop  3 ขยับขวา    เดิม x   =   2.5095596313476562
            [2.5202677249908447, -1.7775897979736328, 0.2, 0.0, 0.0, 0.0, -1.57],   # Drop  4 ขยับขวา    เดิม x   =   2.5202677249908447
            [0.9282695055007935, -2.182033796310425, 0.2, 0.0, 0.0, 0.0, -3.14],    # Drop  5 ขยับลง     เดิม y   =   -2.162033796310425
            [0.6094448566436768, -2.1804385375976562, 0.2, 0.0, 0.0, 0.0, -3.14],   # Drop  6 ขยับลง     เดิม y   =   -2.1604385375976562
            [-0.013216020539402962, 0.02136564441025257, 0.2, 0.0, 0.0, 0.0, 0.0]   # Start 7
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
            q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0,0, pose[6])
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



