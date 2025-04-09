import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from tello_msgs.srv import TelloAction

class DroneMissionNode(Node):
    def __init__(self):
        super().__init__('drone_mission_node')
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ArUco config
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.marker_length = 0.10  #meters

        #intrinsic params
        self.camera_matrix = np.array([
            [921.170702, 0.0,459.904354],
            [0.0,919.018377, 351.238301],
            [0.0,0.0,1.0]
        ], dtype=np.float32)
        self.dist_coeffs = np.array([[-0.033458, 0.105152, 0.001256, -0.006647, 0.0]], dtype=np.float32)


        # state machine setup
        self.state = 'TAKEOFF'
        self.state_start_time = self.get_clock().now().seconds_nanoseconds()[0]

        # service client for takeoff and landing
        self.tello_cli = self.create_client(TelloAction, '/tello_action')
        while not self.tello_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /tello_action service...')

    def set_state(self, new_state):
        self.get_logger().info(f"STATE: {self.state} = {new_state}")
        self.state = new_state
        self.state_start_time = self.get_clock().now().seconds_nanoseconds()[0]

    def time_in_state(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        return now - self.state_start_time

    def send_tello_command(self, cmd):
        req = TelloAction.Request()
        req.cmd = cmd
        self.get_logger().info(f'Sending command: {cmd}')
        self.tello_cli.call_async(req)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        if self.state == 'TAKEOFF':
            self.send_tello_command('takeoff')
            time.sleep(2)
            self.set_state('SEARCH_MARKER_1') #when the state machine starts-> search for marker 1

        elif self.state.startswith('SEARCH_MARKER'):
            target_id = int(self.state[-1]) #id of the marker to search
            if ids is not None and target_id in ids.flatten():
                self.get_logger().info(f"Found marker {target_id}")
                twist = Twist()
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.set_state(f'APPROACH_MARKER_{target_id}')
            else:
                self.rotate()

        elif self.state.startswith('APPROACH_MARKER'):
            target_id = int(self.state[-1])
            if ids is not None and target_id in ids.flatten():
                idx = list(ids.flatten()).index(target_id)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [corners[idx]], self.marker_length, self.camera_matrix, self.dist_coeffs)

                tvec = tvecs[0][0]
                x_offset = tvec[0]
                z_dist = tvec[2]

                twist = Twist()
                if abs(x_offset) > 0.05:
                    twist.angular.z = -0.3 if x_offset > 0 else 0.3
                elif z_dist > 0.5:
                    twist.linear.x = 0.2
                else:
                    self.get_logger().info(f"Positioned at marker {target_id}")
                    twist.linear.x = 0.0
                    if target_id == 1:
                        self.set_state('TURN_LEFT')
                    elif target_id == 2:
                        self.set_state('TURN_RIGHT')
                    elif target_id == 3:
                        self.set_state('LAND')
                self.cmd_pub.publish(twist)

        elif self.state == 'TURN_LEFT':
            if self.time_in_state() < 3:
                twist = Twist()
                twist.angular.z = 0.5
                self.cmd_pub.publish(twist)
            else:
                self.set_state('FORWARD_1')

        elif self.state == 'TURN_RIGHT':
            if self.time_in_state() < 3:
                twist = Twist()
                twist.angular.z = -0.5
                self.cmd_pub.publish(twist)
            else:
                self.set_state('FORWARD_2')

        elif self.state == 'FORWARD_1' or self.state == 'FORWARD_2':
            if self.time_in_state() < 3:
                twist = Twist()
                twist.linear.x = 0.3
                self.cmd_pub.publish(twist)
            else:
                next_marker = 'MARKER_2' if self.state == 'FORWARD_1' else 'MARKER_3'
                self.set_state(f'SEARCH_{next_marker}')

        elif self.state == 'LAND':
            self.send_tello_command('land')
            self.set_state('DONE')

        # Visualization
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.imshow("Drone Camera", frame)

        # Emergency feature for immediate landing
        key = cv2.waitKey(1) & 0xFF
        if key == ord('t') or key == ord('T'):
            self.get_logger().warn('Emergency key [T] pressed! Landing immediately.')
            self.send_tello_command('land')
            self.set_state('DONE')


    def rotate(self):
        twist = Twist()
        twist.angular.z = 0.3
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DroneMissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()