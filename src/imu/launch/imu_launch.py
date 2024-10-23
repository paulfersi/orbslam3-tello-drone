from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    ld = LaunchDescription()

    node=Node(
            package='imu',
            # namespace='orbslam3-odometry',
            name='imu',
            executable='imu',
            output = 'screen',
        )

    ld.add_action(node)
    return ld