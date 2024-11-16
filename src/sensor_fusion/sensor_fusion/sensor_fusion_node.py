from kf import KF 
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class SensorFusion(Node):
    def __init__(self):
        super().__init__('drone_info_gui_node')

        self.orbslam_odom_sub = self.create_subscription(
        Odometry,
        '/Odometry/orbSlamOdom',
        self.orbslam_odom_callback,
        10)

        self.imu_sub = self.create_subscription(
        Imu,
        '/imu/data',
        self.imu_callback,
        10)

        self.corrected_odom_pub = self.create_publisher(Odometry,'/Odometry/correctedOdom',10)

    def orbslam_odom_callback(self,msg):
        pass

    def imu_callback(self,msg):
        pass 