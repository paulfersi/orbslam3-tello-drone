from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    ld = LaunchDescription()

    node=Node(
            package='imu_publisher',
            # namespace='orbslam3-odometry',
            name='imu_publisher',
            executable='imu_publisher',
            output = 'screen',
        )

    ld.add_action(node)
    return ld