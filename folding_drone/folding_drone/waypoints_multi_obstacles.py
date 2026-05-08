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

        # Waypoints for the folding drone to pass through a narrow object working:
        # position_0 = [0.0,0.0,-0.08,0.0,0.0,0.0,0]  # x,y,z, yaw, joint_angle, cumulative time, trajectory number (same number means those waypoins will be sent together for trajectory generation)
        # position_0a = [0.0,0.0,-0.08,0.0,0.0,3.0,1]
        # position_1 = [0.0,0.0,-1.5,0.0,0.0,8.0,2]
        # position_2 = [0.0,0.0,-1.5,0.0,1.54,13.0,3]
        # position_3 = [2.2204,0.0,-1.5,0.0,1.54,19.0,3]  
        # position_4 = [2.62450,1.00470,-1.5,0.0,0.0,22.0,4]  
        # position_5 = [3.497,1.44842,-1.5,0.0,0.78,23.34835,4]  
        # position_6 = [4.23208,1.03842,-1.5,0.0,0.0,25.00775,4]  
        # position_7 = [4.30615,-1.15236,-1.5,0.0,0.78,30.02725,4]  
        # position_8 = [5.295322,-1.32183,-1.5,0.0,0.9,31.417225,4]  
        # position_9 = [6.73808,-0.752188,-1.5,0.0,0.78,33.4443,4]  
        # position_10 = [6.852392,0.165,-1.5,0.0,0.0,35.13475,4]  
        # position_11 = [6.852392,0.165,-0.08,0.0,0.0,39.0,5]  

        # self.positions = np.array([position_0,position_0a,position_1,position_2,position_3,position_4,position_5,position_6,position_7,position_8,position_9,position_10,position_11])

        # RRT Trial Waypoints:
        waypoints = [
            [0, 0, 1],
            [-0.000489021510248171, 0.198924321584194, 0.999005262774465],
            [-0.00125258292979038, 0.686575181647925, 0.998860101789821],
            [-0.0268855322443127, 1.02566088528828, 0.994866923428116],
            [-0.0101174835458686, 1.42334203158576, 0.990835193250539],
            [0.0148340268209948, 1.74077257171048, 0.987897887648195],
            [0.153420503505599, 1.93932647145812, 1.01543745074645],
            [0.448668481459841, 2.3022700975893, 0.98299152259845],
            [0.517924433474591, 2.43134595618504, 0.987630692809991],
            [0.851677747576517, 2.76022529215902, 0.986972092685494],
            [1.04202832862762, 3.07106540720408, 1.0317083437083],
            [1.06883387198688, 3.28193200838041, 1.03288529446523],
            [1.10553628272578, 3.65965133605051, 0.98942996313062],
            [1.00128172052273, 4.02168161104249, 0.947061904106107],
            [0.718724833597378, 4.12799175800702, 0.973670265311541],
            [0.368890364810095, 4.29472866396501, 0.983573695796904],
            [-0.0164902899047696, 4.43904404613367, 1.02396911912375],
            [-0.449885100416822, 4.5461845333608, 1.02895415682634],
            [-0.788358621650025, 4.64619060736813, 1.02561517449365],
            [-0.874447391287847, 4.67807786146565, 0.986822810002835],
            [-1.29824051914425, 4.90132961659888, 1.02900210045891],
            [-1.26841941688004, 5.2983756258204, 1.02230020452641],
            [-1.23442507252526, 5.6944976010438, 1.01963053957441],
            [-1.05245593737555, 5.84038055914079, 1.06105803725293],
            [-0.669540153995516, 6.01797013890915, 1.03870418046319],
            [-0.358349100240282, 6.25244893822006, 0.985879917013787],
            [-0.070955750389029, 6.52003768866877, 1.02346944748045],
            [0.188403284449223, 6.87117025623358, 1.04689392085021],
            [0.165, 6.85, 1]
        ]

        # Transform coordinates and calculate cumulative time
        transformed_waypoints = []
        cumulative_time = 5.0
        speed = 0.3  # m/s

        for i in range(len(waypoints)):
            x, y, z = waypoints[i][1], waypoints[i][0], -waypoints[i][2]-0.5
            if i > 0:
                prev_x, prev_y, prev_z = transformed_waypoints[-1][:3]
                distance = math.sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2 + (z - prev_z) ** 2)
                cumulative_time += distance / speed
            transformed_waypoints.append([x, y, z, 0.0, 1.57, cumulative_time, 1])
        transformed_waypoints.insert(0, [0.0, 0.0, -0.08, 0.0, 0.0, 0.0, 0])

        self.positions = np.array(transformed_waypoints)

        self.position_index = 1  # first target point index

        self.time_init = self.get_clock().now().nanoseconds*10**(-9)  # this will be set as 0 seconds

        timer_period = 1.0/100.0                        # seconds - time peried between each message
        self.timer = self.create_timer(timer_period, self.timer_callback)           # creating timer to execute a function at each time_period

    def timer_callback(self):

        time_passed = self.get_clock().now().nanoseconds*10**(-9) - self.time_init
        # print(str(time_passed/self.positions[-1,5]*100) + '%')
        if time_passed > self.positions[self.position_index-1,5] and self.position_index < len(self.positions):

            msg = FDWaypoint()

            if self.positions[self.position_index,6] != self.positions[self.position_index-1,6]:  # If trajectory number changes, send all the waypoints with the same trajectory number together
                current_trajectory = self.positions[self.position_index,6]
                msg.positions.poses = [Pose(position=Point(x=self.positions[self.position_index-1,0],y=self.positions[self.position_index-1,1],z=self.positions[self.position_index-1,2]))]
                msg.yaws = [self.positions[self.position_index-1,3]]
                msg.joint_angles = [self.positions[self.position_index-1,4]]
                msg.times = [self.positions[self.position_index-1,5]]

                while self.position_index < len(self.positions) and self.positions[self.position_index,6]==current_trajectory:
                    msg.positions.poses.append(Pose(position=Point(x=self.positions[self.position_index,0],y=self.positions[self.position_index,1],z=self.positions[self.position_index,2])))
                    msg.yaws.append(self.positions[self.position_index,3])
                    msg.joint_angles.append(self.positions[self.position_index,4])
                    msg.times.append(self.positions[self.position_index,5])
                    self.position_index += 1
            
            self.setpoints_publisher.publish(msg)
            print(msg)
                    
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