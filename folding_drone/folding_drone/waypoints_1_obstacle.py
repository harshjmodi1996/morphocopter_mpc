# This program creates waypoints for passing through a single narrow object for the folding drone

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
class Waypoints1Obstacle(Node):
    def __init__(self):
        super().__init__('waypoints_1_obstacle')  	                    #initiating the node
        
        # Initiating 0 condition variables: 

        # Initiating required publishers
        self.setpoints_publisher = self.create_publisher(FDWaypoint,'/folding_drone/in/waypoints', 10)  

        self.emergency_home = 0 # Send UAV back to first waypoint inturrepting the current run of waypoints

	# For hardware:
        position_0 = [-1.4,0.0,-0.6,0.0,0.0,0.0]  # x,y,z, yaw, joint_angle, cumulative time
        position_1 = [-1.4,0.0,-0.6,0.0,0.0,20.0]  
        position_2 = [-1.4,0.0,-1.35,0.0,0.0,30.0]
        position_3 = [-1.4,0.0,-1.35,0.0,0.0,38.0]
        position_4 = [-1.4,0.0,-1.35,0.0,1.54,48.0]
        position_5 = [1.4,0.0,-1.35,0.0,1.54,55.0]
        position_6 = [1.4,0.0,-0.2,0.0,0.0,65.0]
        
        # For simulation:
        position_0 = [0.0,0.0,-0.6,0.0,0.0,0.0]  # x,y,z, yaw, joint_angle, cumulative time
        position_1 = [0.0,0.0,-0.6,0.0,0.0,20.0]  
        position_2 = [0.0,0.0,-1.15,0.0,0.0,30.0]
        position_3 = [0.0,0.0,-1.15,0.0,0.0,38.0]
        position_4 = [0.0,0.0,-1.15,0.0,1.54,48.0]
        position_5 = [2.8,0.0,-1.15,0.0,1.54,55.0]
        position_6 = [2.8,0.0,-0.2,0.0,0.0,65.0]

        self.positions = np.array([position_0,position_1,position_2,position_3,position_4,position_5,position_6])

        self.position_index = 0  # first target point index

        self.time_init = self.get_clock().now().nanoseconds*10**(-9)  # this will be set as 0 seconds

        timer_period = 1.0/100.0                        # seconds - time peried between each message
        self.timer = self.create_timer(timer_period, self.timer_callback)           # creating timer to execute a function at each time_period

    def timer_callback(self):

        time_passed = self.get_clock().now().nanoseconds*10**(-9) - self.time_init
        if self.emergency_home ==0:   # only run if previously was not on emergency
            csvreader = csv.reader(open(home_path + '/carl_ws/src/carl_ws_src/folding_drone/folding_drone/emergency_home.csv'))
            for row in csvreader:
                self.emergency_home = float(row[0])

            if self.emergency_home == 1:
                # sending back to position_0 if emergency home commanded
                if self.position_index==0:
                    self.position_index=1 # to avoid it becoming -1, potentially generating trajectory with position_a = last waypoint

                position_a = Pose(position=Point(x=self.positions[self.position_index-1,0],y=self.positions[self.position_index-1,1],z=self.positions[self.position_index-1,2]))
                position_b = Pose(position=Point(x=self.positions[0,0],y=self.positions[0,1],z=0.0))

                msg = FDWaypoint()
                msg.positions.poses=[position_a,position_b]
                msg.yaws = [self.positions[self.position_index-1,3],self.positions[0,3]]
                msg.joint_angles = [self.positions[self.position_index-1,4],self.positions[0,4]]
                msg.times = [0.0,10.0]
                print(msg)
                self.setpoints_publisher.publish(msg)

            else:
                # if not in emergecy_home
                if time_passed > self.positions[self.position_index,5]:
                    self.position_index += 1

                    position_a = Pose(position=Point(x=self.positions[self.position_index-1,0],y=self.positions[self.position_index-1,1],z=self.positions[self.position_index-1,2]))
                    position_b = Pose(position=Point(x=self.positions[self.position_index,0],y=self.positions[self.position_index,1],z=self.positions[self.position_index,2]))

                    msg = FDWaypoint()
                    msg.positions.poses=[position_a,position_b]
                    msg.yaws = [self.positions[self.position_index-1,3],self.positions[self.position_index,3]]
                    msg.joint_angles = [self.positions[self.position_index-1,4],self.positions[self.position_index,4]]
                    msg.times = [self.positions[self.position_index-1,5],self.positions[self.position_index,5]]
                    self.setpoints_publisher.publish(msg)
        else:
            pass
            
        
def main(args=None):
    rclpy.init(args=args)               # initiating rclpy
    waypoints_1_obstacle = Waypoints1Obstacle()  #initializing class

    rclpy.spin(waypoints_1_obstacle)         # running in loop

    #ending the processes (currently, the programme does not reach this point)
    waypoints_1_obstacle.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
