# This program arms the drone and publishes the offbaord control messages to control attitude of the UAV using PX4
# It publishes to the topic /folding_drone/in/actuator_commands and uses it to publish the offboard message to the offboard publisher programme

#Importing Necessary libraries
import rclpy
from rclpy.node import Node
import time
import datetime
import csv
import numpy as np
import math
import os
import sys
home_path = os.path.expanduser('~')
sys.path.insert(1,home_path + '/carl_ws/src/carl_ws_src/folding_drone/folding_drone')  # file path for conversion_functions
# import file
from conversion_functions import *  # Functions defining conversion between rotation matrix, quaternion and euler angles

#Importing Required Messages
from px4_msgs.msg import VehicleAttitude, VehicleCommand, VehicleLocalPosition, InputRc
from std_msgs.msg import Float32MultiArray, Bool, Float32
from geometry_msgs.msg import Point
from folding_drone_msgs.msg import FDActuatorCommands, FDSetpoint, FDOuterloopErrors

# Node class definition
class AttitudeControl(Node):
    def __init__(self):
        super().__init__('attitude_control')  	                    #initiating the node
        
        # Initiating 0 condition variables: 
        self.emergency = False
        self.arming_allowed = False
        self.joint_angle_control_established = False # Updates true when servo motor command to control joint angle has been established
        self.last_position_timestamp = self.get_clock().now().nanoseconds*10**(-9)
        self.roll_command = 0.0
        self.pitch_command = 0.0
        self.yaw_command = 0.0
        self.loop_i = 0
        self.thrust_body = 0.0
        self.integral_error = np.array([[0.0],[0.0],[0.0]])
        self.position_error = np.array([[0.0],[0.0],[0.0]])
        self.velocity_error = np.array([[0.0],[0.0],[0.0]])
        self.acceleration_target = np.array([[0.0],[0.0],[0.0]])
        self.quaternion_command_FRD_Pixhawk = [1.0,0.0,0.0,0.0]
        self.average_thrust = 0.0
        self.average_thrust_count = 0
        self.joint_angle = 0.0
        self.actual_joint_angle = 0.0
        self.roll_error = 0.0
        self.pitch_error = 0.0
        self.yaw_error = 0.0

        # Configuration Properties:

        # Outerloop PID gains

        # Simulation:
        # Kpx = 3.0
        # Kpy = 3.0
        # Kpz = 5.0

        # Tdx = 0.9
        # Tix = 10.0

        # Tdy = 0.9
        # Tiy = 10.0

        # Tdz = 1.1
        # Tiz = 5.0

        #Experiments

        Kpx = 1.6
        Kpy = 1.6
        Kpz = 3.0

        Tdx = 1.3
        Tix = 2.0

        Tdy = 1.3
        Tiy = 2.0

        Tdz = 1.3
        Tiz = 2.0

        self.kp_diag = np.array([[Kpx],[Kpy],[Kpz]])
        self.kv_diag = np.array([[Kpx*Tdx],[Kpy*Tdy],[Kpz*Tdz]])
        self.ki_diag = np.array([[Kpx/Tix],[Kpy/Tiy],[Kpz/Tiz]])

        self.kp_diag_modified = self.kp_diag.copy()
        self.kv_diag_modified = self.kv_diag.copy()
        self.ki_diag_modified = self.ki_diag.copy()

        self.thrust_limit = 10000.0 #N                  # Maximum thrust to be applied in N
        self.tilt_limit = 30.0 #degree                  # Maximum the UAV is allowed to tilt
        self.tilt_limit = self.tilt_limit*math.pi/180.0 # converting to radians (do not edit this)
        self.mass = 1.65 # kg                            # Mass of the UAV (Experiments)
        # self.mass = 2.05 # kg                            # Mass of the UAV (Simulation)
        self.g = 9.81   # m/s^2                         # Gravitational acceleration (absolute value)
        self.innerloop_rate_multiplier = 4              # How fast the innerloop works compared to outerloop (x times)
        timer_period = 1.0/120.0                        # seconds - time peried between each message
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=10)

        # Initiating required publishers
        self.actuator_command_publisher = self.create_publisher(FDActuatorCommands, '/folding_drone/in/actuator_commands', 10)  
        self.arming_allowed_publisher = self.create_publisher(Bool,'/folding_drone/in/arming_allowed',1)
        # Initiating required subscriptions
        # self.vehicle_attitude_subscription = self.create_subscription(VehicleAttitude,'/fmu/out/vehicle_attitude',self.vehicle_attitude_callback, qos_profile=qos_policy) 
        self.setpoint_subscription = self.create_subscription(FDSetpoint, '/folding_drone/in/setpoint', self.setpoint_callback, 10) 
        self.setpoint_subscription = self.create_subscription(FDOuterloopErrors, '/folding_drone/out/outerloop_errors', self.outerloop_errros_callback, 10) 
        self.input_rc_subscription = self.create_subscription(InputRc,'/fmu/out/input_rc',self.input_rc_callback, qos_profile=qos_policy)
        self.actual_joint_angle_subscriptions = self.create_subscription(Float32, '/folding_drone/out/actual_joint_angle', self.actual_joint_angle_callback, qos_profile=qos_policy)

        current_datetime = str(datetime.datetime.now())
        self.filename=home_path+"/carl_ws/log/custom_logs/folding_drone/log_files/attitude_controller"+current_datetime[0:4]+current_datetime[5:7]+current_datetime[8:10]+"_"+current_datetime[11:13]+current_datetime[14:16]+current_datetime[17:19]+".csv"

        with open(self.filename, 'w') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['Time','Epx','Epy','Epz','Evx','Evy','Evz','EIx','EIy','EIz','Roll Command(rad)','Pitch Command (rad)','Yaw Command (rad)','Thrust Physical (N) x','Thrust Command (scaled)','Joint Angle (rad)','kpx:',self.kp_diag_modified[0,0],'kvx: ',self.kv_diag_modified[0,0],'kix: ',self.ki_diag_modified[0,0],'kpy: ',self.kp_diag_modified[1,0],'kvy: ',self.kv_diag_modified[1,0],'kiy: ',self.ki_diag_modified[1,0],'kpz: ',self.kp_diag_modified[2,0],'kvz: ',self.kv_diag_modified[2,0],'kiz: ',self.ki_diag_modified[2,0]])

        # Buffered logging: keep file handle open, flush every N iterations
        self._log_file = open(self.filename, 'a')
        self._log_writer = csv.writer(self._log_file)
        self._log_counter = 0
        self._log_flush_interval = 120  # flush once per second at 120 Hz

        self.timer = self.create_timer(timer_period, self.timer_callback)

    #================================================================= TIMER CALLBACK ===========================================================#

    def timer_callback(self):
        self.position_estimate_health_check()
        self.outerloop()
        msg = FDActuatorCommands()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = True ###########
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False  #########
        msg.attitude_quaternions = self.quaternion_command_FRD_Pixhawk.copy()
        msg.thrust_body = self.thrust_body
        msg.joint_angle = self.actual_joint_angle
        self.actuator_command_publisher.publish(msg)

    def position_estimate_health_check(self):
        if (not self.emergency) and (self.get_clock().now().nanoseconds*10**(-9)-self.last_position_timestamp > 3.0):
            self.get_logger().info('========= Position Estimate not Recieved in last 3 s, emergency landing ==========')
            self.emergency = True

    #================================================================== CONTROL LOOPS ============================================================#

    def outerloop(self):
        self.timestamp = self.get_clock().now().nanoseconds*1e-9
        if self.actual_joint_angle > 1.20:
            self.kp_diag_modified[1,0] = self.kp_diag[1,0]
            self.kv_diag_modified[1,0] = self.kv_diag[1,0]
        else:
            self.kp_diag_modified[:] = self.kp_diag
            self.kv_diag_modified[:] = self.kv_diag
        # Element-wise multiply instead of matmul on diagonal matrices
        self.acceleration_command = (self.acceleration_target
                                     + self.kp_diag_modified * self.position_error
                                     + self.kv_diag_modified * self.velocity_error
                                     + self.ki_diag_modified * self.integral_error)
        
        self.roll_command = max(min((1/-self.g)*(self.acceleration_command[0,0]*math.sin(self.yaw_command)-self.acceleration_command[1,0]*math.cos(self.yaw_command)),self.tilt_limit),-self.tilt_limit)
        self.pitch_command = max(min((1/-self.g)*(self.acceleration_command[0,0]*math.cos(self.yaw_command)+self.acceleration_command[1,0]*math.sin(self.yaw_command)),self.tilt_limit),-self.tilt_limit)
        self.command_quaternions_FRD_Joint = rpy2q(self.roll_command,self.pitch_command,self.yaw_command)
        if not self.emergency:  # if emergency landing is not activated by RC switch
            self.quaternion_command_FRD_Pixhawk = convert_rotation_back(self.actual_joint_angle,self.command_quaternions_FRD_Joint)
            # print(self.quaternion_command_FRD_Pixhawk)
            self.thrust_N = max(min(self.mass*(self.g-self.acceleration_command[2,0]),self.thrust_limit),0.0)
            if self.actual_joint_angle<1.20: # 1.2 for hardware
                # self.thrust_body = max(round((-(self.thrust_N-3.75))/39.2,4),-0.9) # Simulation
                self.thrust_body = max(round((-(self.thrust_N))/32.5,4),-0.9) # Experiments
            else:
                 # self.thrust_body = max(round((-(self.thrust_N-3.75))/39.2,4),-0.9) # Simulation
                self.thrust_body = max(round((-(self.thrust_N))/32.5*1.10,4),-0.9)  # to compensate for propeller interaction caused reduced thrust
            self.average_thrust = ((self.average_thrust*self.average_thrust_count)+self.thrust_body)/(self.average_thrust_count+1)
            self.average_thrust_count+=1
        else:
            self.emergency_land()

        # Buffered CSV write — no open/close per iteration
        self._log_writer.writerow([self.timestamp,self.position_error[0,0],self.position_error[1,0],self.position_error[2,0],self.velocity_error[0,0],self.velocity_error[1,0],self.velocity_error[2,0],self.integral_error[0,0],self.integral_error[1,0],self.integral_error[2,0],self.roll_command,self.pitch_command,self.yaw_command,self.thrust_N,self.thrust_body,self.actual_joint_angle])
        self._log_counter += 1
        if self._log_counter >= self._log_flush_interval:
            self._log_file.flush()
            self._log_counter = 0

    def emergency_land(self):
        self.get_logger().info('\n\n\n\n\n EMERGENCY LANDING \n\n\n\n\n')
        self.quaternion_command_FRD_Pixhawk = [1.0,0.0,0.0,0.0]
        self.thrust_body = self.average_thrust*0.95

    #============================================================ SUBSCRIPTION CALL BACKS ==================================================#

    # def vehicle_attitude_callback(self,msg):
    #     # receiving vehicle attitude
    #     self.vehicle_attitude_timestamp = msg.timestamp
    #     self.vehicle_attitude = msg.q
    #     self.vehicle_attitude_quaternion_FRD_Joint = convert_rotation(self.joint_angle,msg.q)
    #     self.rotation_matrix_FRD_Joint = q2rotmat(self.vehicle_attitude_quaternion_FRD_Joint).copy()
    #     [self.roll,self.pitch,self.yaw] = q2rpy(self.vehicle_attitude_quaternion_FRD_Joint)
    #     # self.innerloop_error_calculation()

    def setpoint_callback(self,msg):
        self.yaw_command = msg.yaw
        self.joint_angle = msg.joint_angle

    def outerloop_errros_callback(self,msg):
        self.last_position_timestamp = self.get_clock().now().nanoseconds*10**(-9)  # when last position estimate was received
        if self.joint_angle_control_established:
            self.arming_allowed = True
            msg1 = Bool()
            msg1.data = True
            self.arming_allowed_publisher.publish(msg1)

        # Reuse arrays instead of creating new ones
        self.position_error[0,0] = msg.position_errors.x
        self.position_error[1,0] = msg.position_errors.y
        self.position_error[2,0] = msg.position_errors.z
        self.velocity_error[0,0] = msg.velocity_errors.x
        self.velocity_error[1,0] = msg.velocity_errors.y
        self.velocity_error[2,0] = msg.velocity_errors.z
        self.integral_error[0,0] = msg.integral_errors.x
        self.integral_error[1,0] = msg.integral_errors.y
        self.integral_error[2,0] = msg.integral_errors.z
        self.acceleration_target[0,0] = msg.acceleration_target.x
        self.acceleration_target[1,0] = msg.acceleration_target.y
        self.acceleration_target[2,0] = msg.acceleration_target.z

    def input_rc_callback(self,msg):
        self.emergency = msg.values[5] > 1500 # checking if emergency land toggle has been activated through RC channel 6

    def actual_joint_angle_callback(self,msg):
        self.joint_angle_control_established = True
        self.actual_joint_angle = msg.data

    #================================================================= MAIN FUNCTION ========================================================#
        
def main(args=None):
    rclpy.init(args=args)               # initiating rclpy
    attitude_control = AttitudeControl()  #initializing class
    rclpy.spin(attitude_control)         # running in loop

    #ending the processes (currently, the programme does not reach this point)
    attitude_control.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
