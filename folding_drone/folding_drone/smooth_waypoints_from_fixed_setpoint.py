# This program generates 2 waypoints: one for current location and one for target setpoint

#Importing Necessary libraries
import rclpy
from rclpy.node import Node
import time
import numpy as np
import math
import csv
import sys
import os
home_path = os.path.expanduser('~')
sys.path.insert(1,home_path + '/carl_ws/src/carl_ws_src/folding_drone/folding_drone')  # file path for conversion_functions

#Importing Required Messages
from folding_drone_msgs.msg import FDWaypoint
from geometry_msgs.msg import Point, Pose
from px4_msgs.msg import VehicleLocalPosition

# Node class definition
class SmoothWaypointsFromFixedSetpoint(Node):
    def __init__(self):
        super().__init__('smooth_waypoints_from_fixed_setpoint')  	                    #initiating the node
        
        # Initiating 0 condition variables: 
        self.desired_nominal_linear_velocity = 1.0 # m/s
        self.current_position = Pose(position=Point(x = 0.0, y = 0.0, z = 0.0))
        self.vehicle_velocity_direct = [0.0,0.0,0.0]

        # Initiating required publishers

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=10)

        self.waypoints_publisher = self.create_publisher(FDWaypoint,'/fodling_drone/in/waypoints', 10)  
        self.vehicle_local_position_subscription = self.create_subscription(VehicleLocalPosition,'/folding_drone/out/vehicle_local_position',self.vehicle_local_position_callback, qos_profile=qos_policy)  

        timer_period = 1.0/10.0                        # seconds - time peried between each message
        self.timer = self.create_timer(timer_period, self.timer_callback)           # creating timer to execute a function at each time_period

    def timer_callback(self):

        csvreader = csv.reader(open(home_path + '/carl_ws/src/carl_ws_src/folding_drone/folding_drone/setpoint.csv'))
        for row in csvreader:
            target_position = Pose(position=Point(x = float(row[0]), y = float(row[1]), z = float(row[2])))
            yaw = float(row[3])
            joint_angle = min(max(float(row[4]),0.0),1.57)


        position_0 = self.current_position
        position_1 = target_position

        euclidean_distance = ((position_0.position.x-position_1.position.x)**2.0 + (position_0.position.y-position_1.position.y)**2.0 + (position_0.position.z-position_1.position.z)**2.0)**0.5
        time_required = euclidean_distance/self.desired_nominal_linear_velocity

        msg = FDWaypoint()
        msg.positions.poses=[self.current_position,target_position]
        msg.initial_velocities = self.vehicle_velocity_direct.copy()
        msg.final_velocities = [0.0,0.0,0.0]
        msg.yaws = [yaw,yaw]
        msg.joint_angles = [joint_angle,joint_angle]
        msg.times = [0.0,time_required]
        self.waypoints_publisher.publish(msg)


    def vehicle_local_position_callback(self,msg):
        # receiving vehicle attitude
        self.vehicle_position_timestamp = msg.timestamp
        self.current_position = Pose(position=Point(x = msg.x, y = msg.y, z = msg.z))
        self.vehicle_velocity_direct = [msg.vx,msg.vy,msg.vz]
        # self.get_logger().info('Current Position: '+str(self.vehicle_position.T))

        
def main(args=None):
    rclpy.init(args=args)               # initiating rclpy
    smooth_waypoints_from_fixed_setpoint = SmoothWaypointsFromFixedSetpoint()  #initializing class

    rclpy.spin(smooth_waypoints_from_fixed_setpoint)         # running in loop

    #ending the processes (currently, the programme does not reach this point)
    smooth_waypoints_from_fixed_setpoint.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()