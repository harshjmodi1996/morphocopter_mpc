from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='morphocopter_planning',
        #     executable='acados_mpc',
        # ),
        Node(
            package='morphocopter_planning',
            executable='mpc_trajectory_to_setpoint',
        ),
        # Node(
        #     package='morphocopter_planning',
        #     executable='lidar_data_forwarder',
        # ),
        # Node(
        #     package='morphocopter_planning',
        #     executable='lidar_data_processor',
        # ),
        
        # if lidar_data_forwarder and lidar_data_processor are commented, they must be running from rviz lidar viewer launch file
        Node(
            package='folding_drone',
            executable='vehicle_local_position_update',
        ),
        Node(
            package='folding_drone',
            executable='outerloop_error_calculations',
        ),
    ])
