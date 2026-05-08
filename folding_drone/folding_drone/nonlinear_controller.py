# This program arms the drone and publishes the offbaord control messages to control the UAV via torque and thrust setpoint 
# It publishes to the topic /folding_drone/in/actuator_commands and uses it to publish the offboard message to the offboard publisher programme

#Importing Necessary libraries
import rclpy
from rclpy.node import Node
import time
import datetime
import numpy as np
import math
import os
import sys
import csv
home_path = os.path.expanduser('~')
sys.path.insert(1,home_path + '/carl_ws/src/carl_ws_src/folding_drone/folding_drone')  # file path for conversion_functions
# import file
from conversion_functions import *  # Functions defining conversion between rotation matrix, quaternion and euler angles

#Importing Required Messages
from px4_msgs.msg import VehicleAttitude, VehicleCommand, VehicleLocalPosition, VehicleOdometry, InputRc, VehicleAngularVelocity
from std_msgs.msg import Float32MultiArray, Bool, Float32
from geometry_msgs.msg import Point
from folding_drone_msgs.msg import FDActuatorCommands, FDSetpoint, FDOuterloopErrors

def calculate_angular_velocity(R1, R2, dt):
    # Convert the rotation matrices to quaternions
    q1 = np.array(rotmat2q(R1))
    q2 = np.array(rotmat2q(R2))
    # Compute the difference quaternion
    q_diff = quaternion_multiply(q2, quaternion_inverse(q1))
    # Extract the vector part of the quaternion
    v = q_diff[1:]
    # Calculate the rotation angle and normalize the vector
    theta = 2 * math.atan2(np.linalg.norm(v), q_diff[0])
    #print(v)
    if np.linalg.norm(v)!=0.0:
        v_normalized = v / np.linalg.norm(v)
    else:
        v_normalized=v                                     ############################################################## could be dangerous
    # Calculate the angular velocity vector
    omega = theta * v_normalized / dt
    return np.array([[omega[0]],[omega[1]],[omega[2]]])

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z])

def quaternion_inverse(q):
    w, x, y, z = q
    q_conjugate = np.array([w, -x, -y, -z])
    q_norm_squared = np.dot(q, q_conjugate)
    return q_conjugate / q_norm_squared

# Node class definition
class NonlinearController(Node):
    def __init__(self):
        super().__init__('nonlinear_controller')  	                    #initiating the node
        
        # Initiating 0 condition variables: 
        self.emergency = False
        self.arming_allowed = False
        self.joint_angle_control_established = False   # Updates true when actual_joint_angle message is received
        self.last_position_timestamp = self.get_clock().now().nanoseconds*10**(-9)
        self.yaw_setpoint = 0.0
        self.loop_i = 2
        self.printer_i = 0
        self.thrust_body = 0.0
        self.position_error = np.array([[0.0],[0.0],[0.0]])
        self.velocity_error = np.array([[0.0],[0.0],[0.0]])
        self.integral_error = np.array([[0.0],[0.0],[0.0]])
        self.acceleration_target = np.array([[0.0],[0.0],[0.0]])
        self.quaternion_command_FRD_Pixhawk = [1.0,0.0,0.0,0.0]
        self.average_thrust = 0.0
        self.average_thrust_count = 0.0
        self.joint_angle = 0.0
        self.actual_joint_angle = 0.0
        self.actual_joint_angle = 0.0
        self.time_inner_prev1 = self.get_clock().now().nanoseconds*10**(-9)
        self.time_outer_prev1 = self.get_clock().now().nanoseconds*10**(-9)
        self.actual_joint_angle = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.zb_error_angle_correction_time = 999999.0
        self.target_angular_speed = 0.5*math.pi/180.0   # rad/s - desired angular speed to correct the angular errors
        self.Rdes = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
        self.Rdes_prev1 = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
        self.rotation_matrix_FRD_Joint = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
        self.er_int = np.array([[0.0],[0.0],[0.0]])
        self.w_target_filtered = np.array([[0.0],[0.0],[0.0]])
        self.angular_velocity_FRD_Joint = np.array([[0.0],[0.0],[0.0]])
        self.odometry_angular_velocity_FRD_Joint = np.array([[0.0],[0.0],[0.0]])
        self.ew_lowpass = np.array([[0.0],[0.0],[0.0]])
        self.fdes = np.array([[0.0],[0.0],[0.0]])
        self.zb = np.array([[0.0],[0.0],[0.0]])
        self.simulation = False # used to name the log file appropriately
        self.km = 0.055 # reaction moment/thrust ratio of motor_prop set (can be calculated experimentally) - property of the prop+motor set
        self.tilt = 25*math.pi/180.0 # physical tilt of the motor in the design (done for roll control in bicopter configuration)
        self.m_factor = 0.03  # for distance compensation due to joint rotation - if above value is not possible to determine
        self.l =  0.242           # (m) distance from center to a motor
        self.Ixx_u = 0.1625 # X 10^6 g mm^2 - Approximate moment of inertia of upper arm about axis aligned to its length and origin at COM of whole drone
        self.Iyy_u = 7.2470 # X 10^6 g mm^2 - Approximate moment of inertia of upper arm perpendicular to its length and origin at COM of whole drone
        self.Ixx_l = 2.7081 # X 10^6 g mm^2 - Approximate moment of inertia of lower arm about axis aligned to its length and origin at COM of whole drone
        self.Iyy_l = 10.8659 # X 10^6 g mm^2 - Approximate moment of inertia of lower arm perpendicular to its length and origin at COM of whole drone


        # Configuration Properties:

        # Outerloop PID gains
        self.kp_outer = np.array([[5.0,0.0,0.0],
                                  [0.0,5.0,0.0],
                                  [0.0,0.0,7.0]])
        self.kv_outer = np.array([[7.0,0.0,0.0],
                                  [0.0,7.0,0.0],
                                  [0.0,0.0,6.5]])
        self.ki_outer = np.array([[0.2,0.0,0.0],
                                  [0.0,0.2,0.0],
                                  [0.0,0.0,1.0]])

        # Innerloop PID gains
        self.kp_inner = np.array([[0.5,0.0,0.0],
                                  [0.0,0.5,0.0],
                                  [0.0,0.0,0.2]])
        self.kv_inner = np.array([[0.075,0.0,0.0],
                                  [0.0,0.075,0.0],
                                  [0.0,0.0,0.1]])
        self.ki_inner = np.array([[0.004,0.0,0.0],
                                  [0.0,0.004,0.0],
                                  [0.0,0.0,0.004]])     #### best working in hardware till 20141122 - still no flight

        # self.kp_inner = np.array([[0.2,0.0,0.0],
        #                           [0.0,0.2,0.0],
        #                           [0.0,0.0,0.3]])
        # self.kv_inner = np.array([[0.04,0.0,0.0],
        #                           [0.0,0.04,0.0],
        #                           [0.0,0.0,0.15]])
        # self.ki_inner = np.array([[0.004,0.0,0.0],
        #                           [0.0,0.004,0.0],
        #                           [0.0,0.0,0.01]])        ##### Simulation working gains

        

        self.thrust_limit = 100000.0 #N                 # Maximum thrust to be applied in N
        self.tilt_limit = 30.0 #degree                  # Maximum the UAV is allowed to tilt
        self.tilt_limit = self.tilt_limit*math.pi/180.0 # converting to radians (do not edit this)
        self.mass = 1.8 # kg                            # Mass of the UAV
        self.g = 9.81   # m/s^2                         # Gravitational acceleration (absolute value)
        self.innerloop_rate_multiplier = 3              # How fast the innerloop works compared to outerloop (x times)
        timer_period = 1.0/300.0                        # seconds - time peried between each message
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=10)

        # Initiating required publishers
        self.actuator_command_publisher = self.create_publisher(FDActuatorCommands, '/folding_drone/in/actuator_commands', 10)  
        self.arming_allowed_publisher = self.create_publisher(Bool,'/folding_drone/in/arming_allowed',1)
        # Initiating required subscriptions
        # self.vehicle_attitude_subscription = self.create_subscription(VehicleAttitude,'/fmu/out/vehicle_attitude',self.vehicle_attitude_callback, qos_profile=qos_policy) 
        self.vehicle_odometry_subscription = self.create_subscription(VehicleOdometry,'/fmu/out/vehicle_odometry',self.vehicle_odometry_callback, qos_profile=qos_policy)
        self.vehicle_angular_velocity_subscription = self.create_subscription(VehicleAngularVelocity,'/fmu/out/vehicle_angular_velocity',self.vehicle_angular_velocity_callback, qos_profile=qos_policy)
        self.input_rc_subscription = self.create_subscription(InputRc,'/fmu/out/input_rc',self.input_rc_callback, qos_profile=qos_policy)
        self.setpoint_subscription = self.create_subscription(FDSetpoint, '/folding_drone/in/setpoint', self.setpoint_callback, 10) 
        self.setpoint_subscription = self.create_subscription(FDOuterloopErrors, '/folding_drone/out/outerloop_errors', self.outerloop_errros_callback, 10) 
        self.actual_joint_angle_subscriptions = self.create_subscription(Float32, '/folding_drone/out/actual_joint_angle', self.actual_joint_angle_callback, qos_profile=qos_policy)
        
        current_datetime = str(datetime.datetime.now())
        self.filename=home_path+"/carl_ws/log/custom_logs/folding_drone/log_files/nonlinear_controller_"+current_datetime[0:4]+current_datetime[5:7]+current_datetime[8:10]+"_"+current_datetime[11:13]+current_datetime[14:16]+current_datetime[17:19]+".csv"

        with open(self.filename, 'w') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['Time','Epx','Epy','Epz','Evx','Evy','Evz','EIx','EIy','EIz','FDesx','FDesy','FDesz','Torque x','Torque y','Torque z','u1','Thrust','er_roll','er_pitch','er_yaw','ew_roll','ew_roll_lowpass','ew_pitch','ew_pitch_lowpass','ew_yaw','ew_yaw_lowpass','target angular vel roll','target angular vel pitch','target angular vel yaw','odometry angular velocity x','odometry angular velocity y','odometry angular velocity z','vehicle angular velocity x','vehicle angular velocity y','vehicle angular velocity z','ei_roll','ei_pitch','ei_yaw','Joint_Angle_Desired','Joint Angle Actual','Roll Estimate (rad)','Pitch Estimate (rad)','Yaw Estimate (rad)','kp_inner roll: '+str(self.kp_inner[0,0]),'kv_inner roll: '+str(self.kv_inner[0,0]),'ki inner roll: '+str(self.ki_inner[0,0]),'kp inner pitch: '+str(self.kp_inner[1,1]),'kv inner pitch: '+str(self.kv_inner[1,1]),'ki inner pitch: '+str(self.ki_inner[1,1]),'kp inner yaw: '+str(self.kp_inner[2,2]),'kv inner yaw: '+str(self.kv_inner[2,2]),'ki inner yaw: '+str(self.ki_inner[2,2])])
            csvfile.close

        self.timer = self.create_timer(timer_period, self.timer_callback)  			# creating timer to execute a function at each time_period

    #================================================================= TIMER CALLBACK ===========================================================#

    def timer_callback(self):
        self.position_estimate_health_check()
        self.innerloop()
        msg = FDActuatorCommands()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = True
        msg.direct_actuator = False
        msg.torque_setpoint = self.torque_setpoint
        msg.thrust_body = self.thrust_body
        msg.joint_angle = self.actual_joint_angle
        self.actuator_command_publisher.publish(msg)

    def position_estimate_health_check(self):
        if (not self.emergency) and (self.get_clock().now().nanoseconds*10**(-9)-self.last_position_timestamp > 3.0):
            self.get_logger().info('========= Position Estimate not Recieved in last 3 s, emergency landing ==========')
            self.emergency = True


    #================================================================== CONTROL LOOPS ============================================================#

    def innerloop(self):
        if self.loop_i==(self.innerloop_rate_multiplier-1):
            self.outerloop()
            self.loop_i=0
        self.loop_i+=1
        dt = self.get_clock().now().nanoseconds*10**(-9) - self.time_inner_prev1



        if (np.linalg.norm(self.zbdes)*np.linalg.norm(np.array([self.zb]).T))!=0:
            zb_error_angle = abs(math.acos(np.dot(self.zbdes.T[0],np.array([self.zb])[0])/(np.linalg.norm(self.zbdes.T)*np.linalg.norm(np.array([self.zb])))))
            self.zb_error_angle_correction_time = zb_error_angle/self.target_angular_speed

        self.w_target_filtered = calculate_angular_velocity(self.rotation_matrix_FRD_Joint,self.Rdes,self.zb_error_angle_correction_time)
        self.w_target_filtered = np.array([[0.0],[0.0],[0.0]])

        self.er_matrix=1.0/2.0*(np.matmul(self.Rdes.T,self.rotation_matrix_FRD_Joint)-np.matmul(self.rotation_matrix_FRD_Joint.T,self.Rdes))                                   # error between desired and current rotation matrix
        self.er=np.array([[-self.er_matrix[2,1]],[-self.er_matrix[0,2]],[-self.er_matrix[1,0]]])      #vee map
        self.er_int=self.er_int+self.er*dt
        self.ew=self.w_target_filtered-self.odometry_angular_velocity_FRD_Joint      
        self.ew_lowpass=0.0*self.ew_lowpass+1.0*self.ew

        ## Calculating desired thrust values
        self.u1 = float(np.clip(np.dot(self.fdes.T[0],self.zb),-self.thrust_limit,0.0))
        # self.get_logger().info('integral_error: '+str(self.integral_error))
        self.u234=np.matmul(self.kp_inner,self.er)+np.matmul(self.kv_inner,self.ew_lowpass)+np.matmul(self.ki_inner,self.er_int)
        dist_compensation_roll = (self.l*math.cos(self.tilt)*math.sin(math.pi/4.0)+self.km*math.sin(self.tilt)*math.cos(math.pi/4.0))/(self.l*math.cos(self.tilt)*math.sin(math.pi/4.0-self.actual_joint_angle/2.0)+self.km*math.sin(self.tilt)*math.cos(math.pi/4.0-self.actual_joint_angle/2.0))
        dist_compensation_pitch = (self.l*math.cos(self.tilt)*math.cos(math.pi/4.0)-self.km*math.sin(self.tilt)*math.sin(math.pi/4.0))/(self.l*math.cos(self.tilt)*math.cos(math.pi/4.0-self.actual_joint_angle/2.0)-self.km*math.sin(self.tilt)*math.sin(math.pi/4.0-self.actual_joint_angle/2.0))
        mi_compensation_roll = ((self.Iyy_u+self.Iyy_l)*(math.sin(math.pi/4.0-self.actual_joint_angle/2.0))**2.0+(self.Ixx_u+self.Ixx_l)*(math.cos(math.pi/4.0-self.actual_joint_angle/2.0))**2.0)/((self.Iyy_u+self.Iyy_l)*(math.sin(math.pi/4.0))**2.0+(self.Ixx_u+self.Ixx_l)*(math.cos(math.pi/4.0))**2.0)
        mi_compensation_pitch = ((self.Iyy_u+self.Iyy_l)*(math.cos(math.pi/4.0-self.actual_joint_angle/2.0))**2.0+(self.Ixx_u+self.Ixx_l)*(math.sin(math.pi/4.0-self.actual_joint_angle/2.0))**2.0)/((self.Iyy_u+self.Iyy_l)*(math.cos(math.pi/4.0))**2.0+(self.Ixx_u+self.Ixx_l)*(math.sin(math.pi/4.0))**2.0)
        # dist_compensation_pitch = 1.0/1.4
        compensation_matrix = np.array([[dist_compensation_roll*mi_compensation_roll,0.0,0.0],[0.0,dist_compensation_pitch*mi_compensation_pitch,0.0],[0.0,0.0,1.0]])
        self.u234_dist_compensated = np.matmul(compensation_matrix,self.u234)
        self.u234_FRD_Pixhawk = self.u234_dist_compensated.copy() # Doesn't need to be converted, Pixhawk does't care/know where the motors are

        if not self.emergency:  # if emergency landing is not activated by RC switch
            self.torque_setpoint = [float(self.u234_FRD_Pixhawk[0,0]),float(self.u234_FRD_Pixhawk[1,0]),float(self.u234_FRD_Pixhawk[2,0])]
            self.thrust_body = (self.u1+3.75)/32.2
            #print(self.u1)


            self.average_thrust=((self.average_thrust*self.average_thrust_count)+self.thrust_body)/(self.average_thrust_count+1)  # updating the average thrust for emergency
            self.average_thrust_count+=1
        else:
            self.emergency_land()

        self.time_inner_prev1=self.get_clock().now().nanoseconds*10**(-9)
        
        with open(self.filename, 'a') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow([self.time_inner_prev1,self.position_error[0,0],self.position_error[1,0],self.position_error[2,0],self.velocity_error[0,0],self.velocity_error[1,0],self.velocity_error[2,0],self.integral_error[0,0],self.integral_error[1,0],self.integral_error[2,0],self.fdes[0,0],self.fdes[1,0],self.fdes[2,0],self.torque_setpoint[0],self.torque_setpoint[1],self.torque_setpoint[2],self.u1,self.thrust_body,self.er[0,0],self.er[1,0],self.er[2,0],self.ew[0,0],self.ew_lowpass[0,0],self.ew[1,0],self.ew_lowpass[1,0],self.ew[2,0],self.ew_lowpass[2,0],self.w_target_filtered[0,0],self.w_target_filtered[1,0],self.w_target_filtered[2,0],self.odometry_angular_velocity_FRD_Joint[0,0],self.odometry_angular_velocity_FRD_Joint[1,0],self.odometry_angular_velocity_FRD_Joint[2,0],self.odometry_angular_velocity_FRD_Joint[0,0],self.odometry_angular_velocity_FRD_Joint[1,0],self.odometry_angular_velocity_FRD_Joint[2,0],self.er_int[0,0],self.er_int[1,0],self.er_int[2,0],self.joint_angle,self.actual_joint_angle,self.roll,self.pitch,self.yaw])
            csvfile.close
        


    def outerloop(self):
        dt = self.get_clock().now().nanoseconds*10**(-9) - self.time_outer_prev1
        self.fdes = self.mass*(self.acceleration_target+self.g*np.array([[0.0],[0.0],[-1.0]])+np.matmul(self.kp_outer,self.position_error)+np.matmul(self.kv_outer,self.velocity_error)+np.matmul(self.ki_outer,self.integral_error))

        ## Calculting desired unit vectors
        if np.linalg.norm(self.fdes)!=0.0:
            self.zbdes=-self.fdes/np.linalg.norm(self.fdes)
        else:
            self.zbdes=np.array([[0.0],[0.0],[1.0]])

        self.xcdes=np.array([[math.cos(self.yaw_setpoint)],[math.sin(self.yaw_setpoint)],[0.0]])

        #   print(self.xcdes)
        self.ybdes=np.cross(self.zbdes,self.xcdes,axis=0)/np.linalg.norm(np.cross(self.zbdes,self.xcdes,axis=0))
        self.xbdes=np.cross(self.ybdes,self.zbdes,axis=0)


        self.Rdes=np.array([[self.xbdes[0,0],self.ybdes[0,0],self.zbdes[0,0]],[self.xbdes[1,0],self.ybdes[1,0],self.zbdes[1,0]],[self.xbdes[2,0],self.ybdes[2,0],self.zbdes[2,0]]])     # desired rotation matrix

        # self.w_target_nonfiltered=calculate_angular_velocity(self.Rdes_prev1,self.Rdes,dt)

        # if dt>=0.0:
        #     self.w_target_filtered=0.95*self.w_target_filtered+0.05*self.w_target_nonfiltered


        # self.w_target_filtered = np.array([[0.0],[0.0],[0.0]])  # this w_target calculation in this manner was not in the paper, and seems wrong. It has to come from trajectory. So using 0s to just damp the angular movment

        self.Rdes_prev1=self.Rdes.copy()

        self.time_outer_prev1=self.get_clock().now().nanoseconds*10**(-9)

    def emergency_land(self):
        self.torque_setpoint = [0.0,0.0,0.0]
        self.thrust_body = self.average_thrust*0.95

    #============================================================ SUBSCRIPTION CALL BACKS ==================================================#

    # def vehicle_attitude_callback(self,msg):
    #     # receiving vehicle attitude
    #     self.vehicle_attitude_timestamp = msg.timestamp
        

    def vehicle_odometry_callback(self,msg):
        # receiving vehicle attitude
        self.vehicle_odometry_timestamp = msg.timestamp
        self.odometry_angular_velocity_FRD_Pixhawk = np.array([[msg.angular_velocity[0]],[msg.angular_velocity[1]],[msg.angular_velocity[2]]])
        self.odometry_angular_velocity_FRD_Joint = convert_axes(self.actual_joint_angle,self.odometry_angular_velocity_FRD_Pixhawk)
        
        self.vehicle_attitude_quaternion_FRD_Joint = convert_rotation(self.actual_joint_angle,msg.q)
        self.rotation_matrix_FRD_Joint = q2rotmat(self.vehicle_attitude_quaternion_FRD_Joint).copy()
        [self.roll,self.pitch,self.yaw] = q2rpy(self.vehicle_attitude_quaternion_FRD_Joint)
        print(self.roll)
        print(self.pitch)
        print(self.yaw)
        print('---')
        self.zb = self.rotation_matrix_FRD_Joint[:,2]

    def vehicle_angular_velocity_callback(self,msg):
        # receiving vehicle attitude
        self.angular_velocity_FRD_Pixhawk = np.array([[msg.xyz[0]],[msg.xyz[1]],[msg.xyz[2]]])
        self.angular_velocity_FRD_Joint = convert_axes(self.actual_joint_angle,self.angular_velocity_FRD_Pixhawk)

    def setpoint_callback(self,msg):
        self.yaw_setpoint = msg.yaw
        self.joint_angle = msg.joint_angle

    def outerloop_errros_callback(self,msg):
        self.last_position_timestamp = self.get_clock().now().nanoseconds*10**(-9)  # when last position estimate was received
        if self.joint_angle_control_established:
            self.arming_allowed = True
            msg1 = Bool()
            msg1.data = True
            self.arming_allowed_publisher.publish(msg1)

        self.position_error = np.array([[msg.position_errors.x],[msg.position_errors.y],[msg.position_errors.z]])
        self.velocity_error = np.array([[msg.velocity_errors.x],[msg.velocity_errors.y],[msg.velocity_errors.z]])
        self.integral_error = np.array([[msg.integral_errors.x],[msg.integral_errors.y],[msg.integral_errors.z]])
        # acceleration target updation has been added after determining 18/20 success with MPC:
        self.acceleration_target = np.array([[msg.acceleration_target.x],[msg.acceleration_target.y],[msg.acceleration_target.z]])

    def input_rc_callback(self,msg):
        self.emergency = msg.values[5] > 1500 # checking if emergency land toggle has been activated through RC channel 6

    def actual_joint_angle_callback(self,msg):
        self.joint_angle_control_established = True
        self.actual_joint_angle = msg.data


    #================================================================= MAIN FUNCTION ========================================================#
        
def main(args=None):
    rclpy.init(args=args)               # initiating rclpy
    nonlinear_controller = NonlinearController()  #initializing class
    rclpy.spin(nonlinear_controller)         # running in loop

    #ending the processes (currently, the programme does not reach this point)
    nonlinear_controller.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
