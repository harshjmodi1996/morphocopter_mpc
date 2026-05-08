# This program creates offboard messages to test the thrust of the motors

#Importing Necessary libraries
import rclpy
from rclpy.node import Node
import time
import numpy as np
import math
import sys
import os
import csv
home_path = os.path.expanduser('~')
sys.path.insert(1,home_path + '/carl_ws/src/carl_ws_src/folding_drone/folding_drone')  # file path for conversion_functions
# import file
from conversion_functions import *  # Functions defining conversion between rotation matrix, quaternion and euler angles

#Importing Required Messages
from px4_msgs.msg import VehicleAttitude
from folding_drone_msgs.msg import FDActuatorCommands
from std_msgs.msg import Float32, Bool

# Node class definition
class AttitudeControl(Node):
    def __init__(self):
        super().__init__('attitude_control')  	                    #initiating the node
        
        # Initiating 0 condition variables: 
        self.thrust = 0.0
        self.torque = [0.0,0.0,0.0]
        self.vehicle_attitude = [1.0,0.0,0.0,0.0]

        timer_period = 1.0/200.0                        # seconds - time peried between each message
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=10)

        # Initiating required publishers
        self.actuator_command_publisher = self.create_publisher(FDActuatorCommands, '/folding_drone/in/actuator_commands', 10)  
        self.arming_allowed_publisher = self.create_publisher(Bool,'/folding_drone/in/arming_allowed',1)
        
        # Initiating required subscriptions
        # self.vehicle_attitude_subscription = self.create_subscription(VehicleAttitude,'/fmu/out/vehicle_attitude',self.vehicle_attitude_callback, qos_profile=qos_policy) 
        #self.vehicle_attitude_subscription = self.create_subscription(Float32,'/thrust_test',self.thrust_test_callback, 10)

        self.timer = self.create_timer(timer_period, self.timer_callback)  			# creating timer to execute a function at each time_period

    #================================================================= TIMER CALLBACK ===========================================================#

    def timer_callback(self):
        self.thrust_update_from_csv()
        msg = FDActuatorCommands()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = True
        msg.direct_actuator = False
        msg.attitude_quaternions = self.vehicle_attitude.copy()
        msg.thrust_body = self.thrust
        msg.torque_setpoint = self.torque.copy()
        self.actuator_command_publisher.publish(msg)
        msg = Bool()
        msg.data = True
        self.arming_allowed_publisher.publish(msg)
        

   # def vehicle_attitude_callback(self,msg):
        # receiving vehicle attitude
   #     self.vehicle_attitude_timestamp = msg.timestamp
   #     self.vehicle_attitude = msg.q

    def thrust_update_from_csv(self):
        csvreader = csv.reader(open(home_path + '/carl_ws/src/carl_ws_src/folding_drone/folding_drone/thrust.csv'))
        for row in csvreader:
            self.thrust = float(row[0])
            self.torque = [float(row[1]),float(row[2]),float(row[3])]

    def thrust_test_callback(self,msg):
        self.thrust = float(msg.data)
        
        
def main(args=None):
    rclpy.init(args=args)               # initiating rclpy
    attitude_control = AttitudeControl()  #initializing class
    rclpy.spin(attitude_control)         # running in loop

    #ending the processes (currently, the programme does not reach this point)
    attitude_control.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
