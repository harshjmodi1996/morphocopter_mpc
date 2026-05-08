# This program s
# It subscribes to the topic drone outerloop positions from PX4 and uses it to publish the position, velocity and integral errors

#Importing Necessary libraries
import rclpy
from rclpy.node import Node
import time
import datetime
import numpy as np
import os
import sys
import csv
home_path = os.path.expanduser('~')
sys.path.insert(1,home_path + '/carl_ws/src/carl_ws_src/folding_drone/folding_drone')  # file path for conversion_functions

#Importing Required Messages
from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import Point
from folding_drone_msgs.msg import FDSetpoint, FDOuterloopErrors
from std_msgs.msg import Bool

# Node class definition
class OuterloopErrorCalculations(Node):
    def __init__(self):
        super().__init__('outerloop_error_calculations')  	                    #initiating the node
        
        # Initiating 0 condition variables: 
        self.position_target = np.array([[0.0],[0.0],[0.0]])
        self.velocity_target = np.array([[0.0],[0.0],[0.0]])
        self.vehicle_position = np.array([[0.0],[0.0],[200.0]])
        # self.vehicle_position_prev1 = np.array([[0.0],[0.0],[0.0]])
        # self.vehicle_position_prev2 = np.array([[0.0],[0.0],[0.0]])
        # self.vehicle_velocity = np.array([[0.0],[0.0],[0.0]])
        self.vehicle_velocity_direct = np.array([[0.0],[0.0],[0.0]])
        self.vehicle_velocity_direct_lowpass = np.array([[0.0],[0.0],[0.0]])
        self.integral_error = np.array([[0.0],[0.0],[0.0]])
        self.position_error = np.array([[0.0],[0.0],[0.0]])
        self.velocity_error = np.array([[0.0],[0.0],[0.0]])
        self.acceleration_target = Point(x=0.0,y=0.0,z=0.0)   # just to forward acceleration target from trajectory to controller
        self.time_prev1 = self.get_clock().now().nanoseconds*10**(-9)
        self.time_prev2 = self.get_clock().now().nanoseconds*10**(-9)
        self.arming_allowed = False
        self.simulation = False # used to name the log file appropriately
        # self.offset_done = 0

        self.velocity_lowpass_factor = 0.8 # weight for the previous velocity value, the time constant related to this filter is: Ts * factor / (1-factor), loosely related to delay introduced due to filtering, 0.8 decided based on Gemini answer for 50 Hz loop, 2kg drone, approximated filter lag of 0.08 s

        self.spatial_limits_simulation = [200.0,200.0,-50.0]  # x,y,z limits in NED convention considering room size (xy origin at the center of the room)
        self.spatial_limits_experiments = [2.0,2.0,-2.0] # x,y,z limits in NED convention considering room size (xy origin at the center of the room)


        self.spatial_limits = [0.0,0.0,0.0]
        self.spatial_limits[:] = self.spatial_limits_experiments[:] # default experiment spatial limits unless confirmed that it is a simulation

        # configuration parameters:
        self.integral_error_influence_dist = 1.0        # Distance from the target point at which the integral erro calculation can start (AND condition in each direction)
        self.integral_elevation = -0.2                 # Elevation of the UAV at which the integration can start (Remember NED convention)
        self.integral_error_limit = 16.0                 # Maximum Integral Error in each direction


        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=10)

        # Initiating required publishers
        self.outerloop_errors_publisher = self.create_publisher(FDOuterloopErrors, '/folding_drone/out/outerloop_errors', 10)  
        # Initiating required subscriptions
        self.vehicle_local_position_subscription = self.create_subscription(VehicleLocalPosition,'/folding_drone/out/vehicle_local_position',self.vehicle_local_position_callback, qos_profile=qos_policy)  
        self.setpoint_subscription = self.create_subscription(FDSetpoint, '/folding_drone/in/setpoint', self.setpoint_callback, 10) 
        self.arming_allowed_subscription = self.create_subscription(Bool,'/folding_drone/in/arming_allowed',self.arming_allowed_callback,1)
        self.simulation_confirmation = self.create_subscription(Bool,'/folding_drone/in/is_simulation',self.simulation_confirmation_callback, qos_profile=qos_policy)

        current_datetime = str(datetime.datetime.now())
        self.filename=home_path+"/carl_ws/log/custom_logs/folding_drone/log_files/outerloop_error_calculations"+current_datetime[0:4]+current_datetime[5:7]+current_datetime[8:10]+"_"+current_datetime[11:13]+current_datetime[14:16]+current_datetime[17:19]+".csv"

        with open(self.filename, 'w') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['Time','Target x','Target y','Target z','Current x', 'Current y', 'Current z','Target vx','Target vy','Target vz','Current vx Direct', 'Current vy Direct', 'Current vz Direct','acceleration target x','acceleration target y','acceleration target z'])
            csvfile.close

    def error_calculation(self):
        self.position_error = self.position_target - self.vehicle_position
        dt = self.get_clock().now().nanoseconds*10**(-9) - self.time_prev1
        # dt_prev1 = self.time_prev1 - self.time_prev2

        
        self.integral_error_updation_value = np.clip(self.integral_error+self.position_error*dt,-self.integral_error_limit,self.integral_error_limit)
        if abs(self.position_error[0,0])<self.integral_error_influence_dist and abs(self.position_error[1,0])<self.integral_error_influence_dist and abs(self.position_error[2,0])<self.integral_error_influence_dist and self.vehicle_position[2,0]<self.integral_elevation and self.arming_allowed:
            self.integral_error[0,0] = self.integral_error_updation_value[0,0]
            self.integral_error[1,0] = self.integral_error_updation_value[1,0]
        # update z direction integral errors even if vehicle has not took off
        if self.arming_allowed:
            self.integral_error[2,0] = self.integral_error_updation_value[2,0]

        # print(dt)

        # if dt!=0.0 and dt_prev1!=0.0:
        #     self.vehicle_velocity = 0.93*self.vehicle_velocity_prev1 + 0.035*(self.vehicle_position-self.vehicle_position_prev1)/dt + 0.035*(self.vehicle_position_prev1-self.vehicle_position_prev2)/dt_prev1

        self.vehicle_velocity_direct_lowpass = self.velocity_lowpass_factor*self.vehicle_velocity_direct_lowpass +\
                                                (1-self.velocity_lowpass_factor)*self.vehicle_velocity_direct

        # self.velocity_error = self.velocity_target - self.vehicle_velocity
        self.velocity_error = self.velocity_target - self.vehicle_velocity_direct_lowpass          # using directly the velocity provided by simulator/Vicon instead of calculatig from position

        # !!!!!!!!!!!!!!!!!!! Please remove velocity calculation after testing on hardware if it works with direct velocity from simulator/Vicon

        # self.vehicle_velocity_prev1 = self.vehicle_velocity.copy()
        # self.vehicle_position_prev2 = self.vehicle_position_prev1.copy()
        # self.vehicle_position_prev1 = self.vehicle_position.copy()

        # self.time_prev2 = self.time_prev1
        self.time_prev1 = self.get_clock().now().nanoseconds*10**(-9)

        #self.get_logger().info('Position Target: '+str(self.position_target.T))
        #self.get_logger().info('Vehicle Position: '+str(self.vehicle_position.T))
        #self.get_logger().info('---------------------------------------------------------')

        if self.arming_allowed:
            with open(self.filename, 'a') as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerow([self.time_prev1,self.position_target[0,0],self.position_target[1,0],self.position_target[2,0],self.vehicle_position[0,0],self.vehicle_position[1,0],self.vehicle_position[2,0],self.velocity_target[0,0],self.velocity_target[1,0],self.velocity_target[2,0],self.vehicle_velocity_direct[0,0],self.vehicle_velocity_direct[1,0],self.vehicle_velocity_direct[2,0],self.acceleration_target.x,self.acceleration_target.y,self.acceleration_target.z])
                csvfile.close

        self.publish_outerloop_errors()

    def publish_outerloop_errors(self):
        msg = FDOuterloopErrors()
        msg.position_errors.x = self.position_error[0,0]
        msg.position_errors.y = self.position_error[1,0]
        msg.position_errors.z = self.position_error[2,0]
        msg.velocity_errors.x = self.velocity_error[0,0]
        msg.velocity_errors.y = self.velocity_error[1,0]
        msg.velocity_errors.z = self.velocity_error[2,0]
        msg.integral_errors.x = self.integral_error[0,0]
        msg.integral_errors.y = self.integral_error[1,0]
        msg.integral_errors.z = self.integral_error[2,0]
        msg.acceleration_target = self.acceleration_target
        self.outerloop_errors_publisher.publish(msg)

    #============================================================ SUBSCRIPTION CALL BACKS ==================================================#

    def vehicle_local_position_callback(self,msg):
        # receiving vehicle attitude
        self.vehicle_position_timestamp = msg.timestamp
        # if self.vehicle_position[0,0]>1.0 and self.offset_done <=30: 
        #     offset = -0.6
        #     self.offset_done += 1
        # else:
        #     offset = 0.0
        self.vehicle_position = np.array([[msg.x],[msg.y],[msg.z]])

        self.vehicle_velocity_direct = np.array([[msg.vx],[msg.vy],[msg.vz]])
        # self.get_logger().info('Current Position: '+str(self.vehicle_position.T))
        self.error_calculation()
        
    def setpoint_callback(self,msg):
        self.position_target = np.array([[max(min(msg.position.x,self.spatial_limits[0]),-self.spatial_limits[0])],[max(min(msg.position.y,self.spatial_limits[1]),-self.spatial_limits[1])],[min(max(msg.position.z,self.spatial_limits[2]),0.0)]])
        self.velocity_target = np.array([[msg.velocity.x],[msg.velocity.y],[msg.velocity.z]])
        self.acceleration_target = msg.acceleration
        
    def arming_allowed_callback(self,msg):
        # checking if arming is allowed (if position estimate received)
        self.arming_allowed = msg.data

    def simulation_confirmation_callback(self,msg):
        self.simulation = msg.data
        self.spatial_limits[:] = self.spatial_limits_simulation[:]


    #================================================================= MAIN FUNCTION ========================================================#
        
def main(args=None):
    rclpy.init(args=args)               # initiating rclpy
    outerloop_error_calculations = OuterloopErrorCalculations()  #initializing class
    rclpy.spin(outerloop_error_calculations)         # running in loop

    #ending the processes (currently, the programme does not reach this point)
    outerloop_error_calculations.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()