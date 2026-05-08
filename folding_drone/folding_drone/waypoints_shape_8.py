# This program creates waypoints for moving in a diamond shaped trajectory

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

# Node class definition
class WaypointsShape8(Node):
    def __init__(self):
        super().__init__('waypoints_shape_8')  	                    #initiating the node
        
        # Initiating 0 condition variables: 

        # Initiating required publishers
        self.setpoints_publisher = self.create_publisher(FDWaypoint,'/folding_drone/in/waypoints', 10)  

        self.emergency_home = 0 # Send UAV back to first waypoint inturrepting the current run of waypoints

        joint_angle_during_shape = 1.4 # desired joing angle during shape 8 traverse

        position_0 = [0.0,0.0,-0.3,0.001,0.0,0.0,0]  # x,y,z, yaw, joint_angle, cumulative time, shape waypoints flag (0/1)
        position_1 = [0.0,0.0,-0.3,0.001,0.0,20.0,0]  
        position_2 = [0.0,0.0,-1.2,0.001,0.0,39.0,0]
        position_3 = [0.0,0.0,-1.2,0.001,0.0,40.0,0]
        position_4 = [0.0,0.0,-1.2,0.001,joint_angle_during_shape,50.0,0]

        position_5 = [0.8,1.5,-1.2,0.001,joint_angle_during_shape,54.0,1]
        position_6 = [2.0,0.0,-1.2,0.001,joint_angle_during_shape,58.0,1]
        position_7 = [0.8,-1.5,-1.2,0.001,joint_angle_during_shape,62.0,1]
        position_8 = [0.0,0.0,-1.2,0.001,joint_angle_during_shape,66.0,1]
        position_9 = [-0.8,1.5,-1.2,0.001,joint_angle_during_shape,70.0,1]
        position_10 = [-2.0,0.0,-1.2,0.001,joint_angle_during_shape,74.0,1]
        position_11 = [-0.8,-1.5,-1.2,0.001,joint_angle_during_shape,78.0,1]
        position_12 = [0.0,0.0,-1.2,0.001,joint_angle_during_shape,82.0,1]

        position_13 = [0.0,0.0,-1.2,0.001,0.0,86.0,0]
        position_14 = [0.0,0.0,0.0,0.001,0.0,90.0,0]


        self.positions = np.array([position_0,position_1,position_2,position_3,position_4,position_5,position_6,position_7,position_8,position_9,position_10,position_11,position_12,position_13,position_14])

        self.position_index = 1  # first target point index

        self.time_init = self.get_clock().now().nanoseconds*10**(-9)  # this will be set as 0 seconds

        timer_period = 1.0/100.0                        # seconds - time peried between each message
        self.timer = self.create_timer(timer_period, self.timer_callback)           # creating timer to execute a function at each time_period

    def timer_callback(self):

        time_passed = self.get_clock().now().nanoseconds*10**(-9) - self.time_init
        if time_passed > self.positions[self.position_index-1,5]:

            msg = FDWaypoint()

            if self.positions[self.position_index,6] == 1:
                msg.positions.poses = [Pose(position=Point(x=self.positions[self.position_index-1,0],y=self.positions[self.position_index-1,1],z=self.positions[self.position_index-1,2]))]
                msg.yaws = [self.positions[self.position_index-1,3]]
                msg.joint_angles = [self.positions[self.position_index-1,4]]
                msg.times = [self.positions[self.position_index-1,5]]

                while self.positions[self.position_index,6]==1:
                    msg.positions.poses.append(Pose(position=Point(x=self.positions[self.position_index,0],y=self.positions[self.position_index,1],z=self.positions[self.position_index,2])))
                    msg.yaws.append(self.positions[self.position_index,3])
                    msg.joint_angles.append(self.positions[self.position_index,4])
                    msg.times.append(self.positions[self.position_index,5])
                    self.position_index += 1
            else:
                position_a = Pose(position=Point(x=self.positions[self.position_index-1,0],y=self.positions[self.position_index-1,1],z=self.positions[self.position_index-1,2]))
                position_b = Pose(position=Point(x=self.positions[self.position_index,0],y=self.positions[self.position_index,1],z=self.positions[self.position_index,2]))

                msg.positions.poses=[position_a,position_b]
                msg.yaws = [self.positions[self.position_index-1,3],self.positions[self.position_index,3]]
                msg.joint_angles = [self.positions[self.position_index-1,4],self.positions[self.position_index,4]]
                msg.times = [self.positions[self.position_index-1,5],self.positions[self.position_index,5]]

                self.position_index += 1
            
            self.setpoints_publisher.publish(msg)
                    
        else:
            pass
            
        
def main(args=None):
    rclpy.init(args=args)               # initiating rclpy
    waypoints_shape_8 = WaypointsShape8()  #initializing class

    rclpy.spin(waypoints_shape_8)         # running in loop

    #ending the processes (currently, the programme does not reach this point)
    waypoints_shape_8.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
