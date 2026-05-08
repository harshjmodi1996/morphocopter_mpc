import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from folding_drone_msgs.msg import FD4DPoseArray, FD4DPose
import csv
import os

class RRTWaypointLogger(Node):
    def __init__(self):
        super().__init__('rrt_waypoint_logger')
        
        # Define the subscriber
        
        self.subscription = self.create_subscription(
            FD4DPoseArray,
            '/folding_drone/path_planner_waypoints',
            self.waypoint_callback_ompl,
            10)
        
        # Define the CSV file path
        self.csv_file_path = '/home/harshmodi/carl_ws/src/carl_ws_src/morphocopter_planning/acados_mpc/tracks/morphocopter_waypoints.csv'
        
        # Overwrite the previous file when the node is re-run
        with open(self.csv_file_path, mode='w') as file:
            writer = csv.writer(file)
            writer.writerow(['x', 'y', 'z', 'yaw'])

    def waypoint_callback_ompl(self, msg):
        # Append new waypoints to the CSV file
        for row in msg.poses:
            data_for_writing = []
            data_for_writing.append([row.position.x, row.position.y, row.position.z, row.yaw])

            with open(self.csv_file_path, mode='a') as file:
                    writer = csv.writer(file)
                    writer.writerows(data_for_writing)
        self.get_logger().info('Waypoints logged to CSV file')

def main(args=None):
    rclpy.init(args=args)
    rrt_waypoint_logger = RRTWaypointLogger()
    rclpy.spin(rrt_waypoint_logger)
    rrt_waypoint_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()