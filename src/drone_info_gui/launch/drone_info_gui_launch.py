from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_info_gui',  # Replace with your package name
            executable='drone_info_gui_node',  
            name='drone_info_gui_node',
            output='screen',
        )
    ])
