# This program creates offboard messages to test the thrust of the motors

#Importing Necessary libraries
import rclpy
from rclpy.node import Node
import time
import numpy as np
import math

#Importing Required Messages
from folding_drone_msgs.msg import FDWaypoint
from geometry_msgs.msg import Point, Pose

# Node class definition
class WaypointsTrial(Node):
    def __init__(self):
        super().__init__('waypoints_trial')  	                    #initiating the node
        
        # Initiating 0 condition variables: 

        # Initiating required publishers
        self.setpoints_publisher = self.create_publisher(FDWaypoint,'/folding_drone/in/waypoints', 10)  

        position_0 = Pose(position=Point(x=0.0,y=0.0,z=-0.5))
        position_1 = Pose(position=Point(x=0.0,y=0.0,z=-0.5))
        position_2 = Pose(position=Point(x=0.0,y=0.0,z=-1.5))
        position_3 = Pose(position=Point(x=0.0,y=0.0,z=-1.5))
        position_4 = Pose(position=Point(x=0.0,y=0.0,z=-1.5))
        position_5 = Pose(position=Point(x=-2.0,y=-2.0,z=-1.5))
        position_6 = Pose(position=Point(x=-2.0,y=-2.0,z=-1.5))
        position_7 = Pose(position=Point(x=-2.0,y=-2.0,z=-0.2))

        msg = FDWaypoint()
        msg.positions.poses=[position_0,position_1,position_2,position_3,position_4,position_5,position_6,position_7]
        msg.yaws = [0.78,0.78,0.78,0.78,0.78,0.78,0.78,0.78]
        msg.joint_angles = [0.0,0.0,0.0,1.5,1.5,1.5,0.0,0.0]
        msg.times = [5.0,10.0,20.0,30.0,40.0,55.0,60.0,70.0]
        self.setpoints_publisher.publish(msg)


        
def main(args=None):
    rclpy.init(args=args)               # initiating rclpy
    waypoints_trial = WaypointsTrial()  #initializing class

if __name__ == '__main__':
    main()