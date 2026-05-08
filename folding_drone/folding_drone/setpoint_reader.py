# It reads the setpoint values stored in the csv file and publishes to /folding_drone/in/setpoint topic

#Importing Necessary libraries
import rclpy
from rclpy.node import Node
import csv
import sys
import os
home_path = os.path.expanduser('~')
sys.path.insert(1,home_path + '/carl_ws/src/carl_ws_src/folding_drone/folding_drone')  # file path for conversion_functions

#Importing Required Messages
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from folding_drone_msgs.msg import FDSetpoint

# Node class definition
class SetpointReader(Node):
    def __init__(self):
        super().__init__('setpoint_reader')  	                    #initiating the node
        
        

        # Initiating required publishers
        self.setpoint_publisher = self.create_publisher(FDSetpoint, '/folding_drone/in/setpoint', 10)  

        timer_period = 1.0/10.0                        # seconds - time peried between each message
        self.timer = self.create_timer(timer_period, self.timer_callback)           # creating timer to execute a function at each time_period
    
    #============================================================ SUBSCRIPTION CALL BACKS ==================================================#

    def timer_callback(self):
        msg = FDSetpoint()
        csvreader = csv.reader(open(home_path + '/carl_ws/src/carl_ws_src/folding_drone/folding_drone/setpoint.csv'))
        for row in csvreader:
            msg.position = Point(x = float(row[0]), y = float(row[1]), z = float(row[2]))
            msg.yaw = float(row[3])
            msg.joint_angle = min(max(float(row[4]),0.0),1.57)
        self.setpoint_publisher.publish(msg)

    #================================================================= MAIN FUNCTION ========================================================#
        
def main(args=None):
    rclpy.init(args=args)               # initiating rclpy
    setpoint_reader = SetpointReader()  #initializing class
    rclpy.spin(setpoint_reader)         # running in loop

    #ending the processes (currently, the programme does not reach this point)
    setpoint_reader.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()