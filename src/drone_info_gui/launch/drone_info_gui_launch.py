from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():


    waypoint_node = Node(
        name='drone_info_gui',
        package='drone_info_gui',
        executable='drone_info_gui_node',
        output="screen"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation (Gazebo) clock if true'
            ),
            waypoint_node
        ]
    )