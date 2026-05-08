from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='folding_drone',
            executable='thrust_measurement',
        ),
        Node(
            package='folding_drone',
            executable='offboard_publisher',
        )
        
    ])