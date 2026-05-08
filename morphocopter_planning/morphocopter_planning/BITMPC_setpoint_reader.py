# It reads the setpoint values stored in the csv file (generated using BITMPC algorithm) and publishes to /folding_drone/in/setpoint topic

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
class BITMPCSetpointReader(Node):
    def __init__(self):
        super().__init__('BITMPC_setpoint_reader')  	                    #initiating the node
        
        

        # Initiating required publishers
        self.setpoint_publisher = self.create_publisher(FDSetpoint, '/folding_drone/in/setpoint', 10    )  

        timer_period = 1.0/10.0                        # seconds - time peried between each message
        self.timer = self.create_timer(timer_period, self.timer_callback)           # creating timer to execute a function at each time_period
        self.t0 = self.get_clock().now().nanoseconds*10**(-9)
        self.setpoint_index = 0
    
    #============================================================ SUBSCRIPTION CALL BACKS ==================================================#

    def timer_callback(self):
        msg = FDSetpoint()
        # Open the CSV file and read its contents
        with open(home_path + '/carl_ws/src/carl_ws_src/folding_drone/folding_drone/setpoints_BIT_MPC_environment6.csv', 'r') as csvfile:
            csvreader = list(csv.reader(csvfile))  # Convert the reader object to a list

        t1 = self.get_clock().now().nanoseconds * 10**(-9)
        current_time = t1 - self.t0

        # Find the closest timestamp in the first column
        timestamps = [float(row[0]) for row in csvreader]  # Extract the first column as a list of floats
        closest_index = min(range(len(timestamps)), key=lambda i: abs(timestamps[i] - current_time))
        self.setpoint_index = closest_index

        # Populate the FDSetpoint message
        msg.position = Point(
            x=float(csvreader[self.setpoint_index][1]),
            y=float(csvreader[self.setpoint_index][2]),
            z=float(csvreader[self.setpoint_index][3])
        )
        msg.velocity = Point(
            x=float(csvreader[self.setpoint_index][4]),
            y=float(csvreader[self.setpoint_index][5]),
            z=float(csvreader[self.setpoint_index][6])
        )
        msg.acceleration = Point(
            x=float(csvreader[self.setpoint_index][7]),
            y=float(csvreader[self.setpoint_index][8]),
            z=float(csvreader[self.setpoint_index][9])
        )
        msg.yaw = float(csvreader[self.setpoint_index][10])
        msg.joint_angle = min(max(float(csvreader[self.setpoint_index][11]), 0.0), 1.57)

        # Publish the message
        self.setpoint_publisher.publish(msg)

    #================================================================= MAIN FUNCTION ========================================================#
        
def main(args=None):
    rclpy.init(args=args)               # initiating rclpy
    BITMPC_setpoint_reader = BITMPCSetpointReader()  #initializing class
    rclpy.spin(BITMPC_setpoint_reader)         # running in loop

    #ending the processes (currently, the programme does not reach this point)
    BITMPC_setpoint_reader.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()