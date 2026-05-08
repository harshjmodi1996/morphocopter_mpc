# It publishes the PWM value to Servo motor connected to the Raspberry Pi after getting the joint angle value from /folding_drone/in/setpoint

#Importing Necessary libraries
import rclpy
from rclpy.node import Node
import numpy as np
import pigpio
import math
import csv
import sys
import os
home_path = os.path.expanduser('~')
sys.path.insert(1,home_path + '/carl_ws/src/carl_ws_src/folding_drone/folding_drone')  # file path for conversion_functions

#Importing Required Messages
from folding_drone_msgs.msg import FDSetpoint
from std_msgs.msg import Float32, Bool

# Node class definition
class JointAnglePublisher(Node):
    def __init__(self):
        super().__init__('joint_angle_publisher')  	                    #initiating the node
        
        
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=10)

        self.rotation_speed_max = 60.0*math.pi/180.0 # for hardware: 45.0/2.0*math.pi/180.0 # rad/s
        self.angle_min = 0.0    # hardware constrained angle
        self.angle_max = 1.54  # hardware constrained angle
        self.PWM_min = 500      # PWM value of servo corresponding to angle_max, obsolete for 4 bar mechanism based control
        self.PWM_max = 2400     # PWM value of servo corresponding to angle_min, obsolete for 4 bar mechanism based control
        self.joint_angle_old = 0.0
        self.joint_angle_desired = 0.0
        self.pi = pigpio.pi()
        self.setpoints_received = 0 # updating numbers for first few setpooint is received

        self.PWM_data = np.empty([0,2])
        csvreader = csv.reader(open(home_path + '/carl_ws/src/carl_ws_src/folding_drone/folding_drone/PWM_data.csv'))   # reading 4 bar mechanism PWM vs output angle data
        for row in csvreader:
            self.PWM_data = np.append(self.PWM_data , np.array([[float(row[0]),float(row[1])]]),axis=0)

        # Initiating required publishers
        self.actual_joint_angle_publisher = self.create_publisher(Float32, '/folding_drone/out/actual_joint_angle', qos_profile=qos_policy)
        self.setpoint_subscription = self.create_subscription(FDSetpoint, '/folding_drone/in/setpoint', self.setpoint_callback, qos_profile = qos_policy) 
        
        timer_period = 1.0/100.0                        # seconds - time peried between each message
        self.timer = self.create_timer(timer_period, self.timer_callback)           # creating timer to execute a function at each time_period
    
    #============================================================ SUBSCRIPTION CALL BACKS ==================================================#


    def timer_callback(self):
        if self.setpoints_received>9:
            # print(self.joint_angle_desired)
            # print(self.joint_angle_desired_previous)
            if self.joint_angle_desired != self.joint_angle_desired_previous:
                self.change_time = self.get_clock().now().nanoseconds*10**(-9)
                self.joint_angle_old = self.joint_angle_command
                self.change_duration = abs((self.joint_angle_desired-self.joint_angle_old)/self.rotation_speed_max)
                self.joint_angle_desired_previous = self.joint_angle_desired
    
            self.current_time = self.get_clock().now().nanoseconds*10**(-9)
    
            if hasattr(self,'change_time') and self.current_time - self.change_time <= self.change_duration:
                #print((self.current_time-self.change_time)/self.change_duration)
                self.joint_angle_command = self.joint_angle_old+(self.joint_angle_desired-self.joint_angle_old)*(self.current_time-self.change_time)/self.change_duration
            else:
                self.joint_angle_command = self.joint_angle_desired
    
            # servo_PWM = self.PWM_min + (self.PWM_max-self.PWM_min)/(self.angle_max-self.angle_min)*self.joint_angle_command   # this is for linear motor to joint angle relationship
            servo_PWM = float(self.PWM_data[(np.abs(self.PWM_data[:,1] - self.joint_angle_command)).argmin(),0])  # reading required PWM value corresponding to the desired joint angle command based on stored values for 4 bar mechanism
            # self.pi.set_servo_pulsewidth(18, servo_PWM)
            self.actual_joint_angle_publisher.publish(Float32(data = self.joint_angle_command))
        else:
            pass
    
    def setpoint_callback(self,msg):
        if self.setpoints_received < 10:   # this ensures that the initialization is not done at 0 angle, but rather previously saved angle in setpoint.csv
                self.joint_angle_desired_previous = min(max(float(msg.joint_angle),self.angle_min),self.angle_max)
                self.joint_angle_command = self.joint_angle_desired_previous
                self.setpoints_received += 1
        self.joint_angle_desired = min(max(float(msg.joint_angle),self.angle_min),self.angle_max)  # to keep it within hardware limit

        
    #================================================================= MAIN FUNCTION ========================================================#
        
def main(args=None):
    rclpy.init(args=args)               # initiating rclpy
    joint_angle_publisher = JointAnglePublisher()  #initializing class
    rclpy.spin(joint_angle_publisher)         # running in loop

    #ending the processes (currently, the programme does not reach this point)
    joint_angle_publisher.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
