#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

#     tf_node = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='base_link_to_base_laser_ld19',
#         arguments=['0','0','0.18','0','0','0','gazebo_lidar','map']
#     )

    lidar_data_forwarder = Node(
            package='morphocopter_planning',
            executable='lidar_data_forwarder',
            name='lidar_data_forwarder',
    )

    lidar_processor_node = Node(
            package='morphocopter_planning',
            executable='lidar_data_processor',
            name='lidar_data_processor',
    )

    segment_visualizer_node = Node(
            package='morphocopter_planning',
            executable='segment_visualizer_rviz',
            name='segment_visualizer_rviz',
    )

    map_publisher_node = Node(
            package='morphocopter_planning',
            executable='publish_map_for_rviz',
            name='publish_map_for_rviz',
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()

#     ld.add_action(tf_node)
    ld.add_action(lidar_processor_node)
    ld.add_action(segment_visualizer_node)
    ld.add_action(lidar_data_forwarder)
    ld.add_action(map_publisher_node)
    
    return ld