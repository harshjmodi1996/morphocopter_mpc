from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
    	Node(
            package='folding_drone',
            executable='setpoint_reader',
        ),
        # Node(
        #     package='folding_drone',
        #     executable='waypoints_to_setpoints',
        # ),
        Node(
            package='folding_drone',
            executable='vehicle_local_position_update',
        ),
        Node(
            package='folding_drone',
            executable='outerloop_error_calculations',
        ),
    ])
