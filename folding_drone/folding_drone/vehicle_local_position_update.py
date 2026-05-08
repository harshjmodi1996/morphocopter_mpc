# It subscribes to /fmu/out/vehicle_local_position and vicon position topic and publishes the position after necessary conversion to /folding_drone/out/vehicle_local_position

#Importing Necessary libraries
import rclpy
from rclpy.node import Node
import os
import sys

#Importing Required Messages
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry, VehicleAngularVelocity
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
from folding_drone_msgs.msg import FDOrientationNED, FDOrientationAndRate

home_path = os.path.expanduser('~')
sys.path.insert(1,home_path + '/carl_ws/src/carl_ws_src/folding_drone/folding_drone')  # file path for conversion_functions
# import file
from conversion_functions import *  # Functions defining conversion between rotation matrix, quaternion and euler angles

# Node class definition
class VehicleLocalPositionUpdate(Node):
    def __init__(self):
        super().__init__('vehicle_local_position_update')  	                    #initiating the node

        self.vicon_vehicle_local_position = VehicleLocalPosition()
        self.simulation_vehicle_local_position = VehicleLocalPosition()

        # publishing vehicle orientation data for using with lidar scanner
        self.vicon_vehicle_orientation = FDOrientationNED()  # default orientation in NED convention - w,x,y,z format
        self.vicon_vehicle_orientation.q = [1.0, 0.0, 0.0, 0.0]  # default orientation in NED convention - w,x,y,z format
        self.simulation_vehicle_local_orientation = FDOrientationNED()  # default orientation in NED convention - w,x,y,z format
        self.simulation_vehicle_local_orientation.q = [1.0, 0.0, 0.0, 0.0]  # default orientation in NED convention - w,x,y,z format
        self.actual_joint_angle = 0.0 

        self.vicon_odometry_msg = VehicleOdometry()  # publishing vicon data to PX4 directly to aid its position and orientation estimate
        
        
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=10)
        self.simulation = False # checking if user has confirmed that it is a simulation run

        # Initiating required publishers
        self.vehicle_local_position_publisher = self.create_publisher(VehicleLocalPosition, '/folding_drone/out/vehicle_local_position', qos_profile=qos_policy)
        self.vehicle_local_orientation_publisher = self.create_publisher(FDOrientationNED, '/folding_drone/out/vehicle_orientation', qos_profile=qos_policy)   # currently using for lidar
        self.vehicle_local_orientation_and_rate_publisher = self.create_publisher(FDOrientationAndRate, '/folding_drone/out/vehicle_orientation_and_rate', qos_profile=qos_policy)      # currently trying for MPC initial condition

        self.vicon_odometry_publisher = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos_profile=qos_policy)
        
        
        # Initiating required subscriptions
        self.vehicle_local_position_subscription_simulation = self.create_subscription(Odometry,'/model/folding_drone_v2_0/odometry',self.vehicle_local_position_callback_simulation, 1)
        self.vehicle_local_position_subscription_simulation = self.create_subscription(VehicleLocalPosition,'/fmu/out/vehicle_local_position',self.vehicle_local_velocity_callback_simulation, qos_profile = qos_policy)
        self.simulation_confirmation = self.create_subscription(Bool,'/folding_drone/in/is_simulation',self.simulation_confirmation_callback, qos_profile=qos_policy)
        self.vehicle_local_position_subscription_vicon = self.create_subscription(PoseStamped,'/vrpn_mocap/folding_UAV/pose',self.vehicle_local_position_callback_vicon, qos_profile = qos_policy)
        self.vehicle_local_position_subscription_vicon = self.create_subscription(TwistStamped,'/vrpn_mocap/folding_UAV/twist',self.vehicle_local_velocity_callback_vicon, qos_profile = qos_policy)

        self.actual_joint_angle_subscriptions = self.create_subscription(Float32, '/folding_drone/out/actual_joint_angle', self.actual_joint_angle_callback, qos_profile=qos_policy)

        # Subscribing from onboard sensor for simulation
        self.vehicle_odometry_subscription = self.create_subscription(VehicleOdometry,'/fmu/out/vehicle_odometry',self.vehicle_onboard_odometry_callback, qos_profile=qos_policy)


    #============================================================ SUBSCRIPTION CALL BACKS ==================================================#

    # def vehicle_local_position_callback_simulation(self,msg):
    #     # receiving vehicle attitude from simulation
    #     if self.simulation:
    #         msg1 = VehicleLocalPosition()
    #         msg1.timestamp = msg.timestamp
    #         msg1.x = msg.x
    #         msg1.y = msg.y
    #         msg1.z = msg.z
    #         msg1.vx = msg.vx
    #         msg1.vy = msg.vy
    #         msg1.vz = msg.vz
    #         self.vehicle_local_position_publisher.publish(msg1)
    #     else:
    #         pass

    def vehicle_local_position_callback_simulation(self,msg):
        # receiving vehicle attitude from simulation
        if self.simulation:
            self.simulation_vehicle_local_position.x = msg.pose.pose.position.y
            self.simulation_vehicle_local_position.y = msg.pose.pose.position.x
            self.simulation_vehicle_local_position.z = -msg.pose.pose.position.z
            self.vehicle_local_position_publisher.publish(self.simulation_vehicle_local_position)

            self.simulation_vehicle_local_orientation.q = [msg.pose.pose.orientation.w,msg.pose.pose.orientation.y,msg.pose.pose.orientation.x,-msg.pose.pose.orientation.z]
            self.vehicle_local_orientation_publisher.publish(self.simulation_vehicle_local_orientation)
            
        else:
            pass

    def vehicle_local_velocity_callback_simulation(self,msg):
        # receiving vehicle attitude from simulation
        # No ENU -> NED conversion needed since this is coming from PX4
        if self.simulation:
            self.simulation_vehicle_local_position.timestamp = msg.timestamp
            self.simulation_vehicle_local_position.vx = msg.vx
            self.simulation_vehicle_local_position.vy = msg.vy
            self.simulation_vehicle_local_position.vz = msg.vz
        else:
            pass

    
    def vehicle_local_position_callback_vicon(self,msg):
        # receiving vehicle attitude from vicon (converting from ENU to NED)
        self.vicon_vehicle_local_position.x = msg.pose.position.y
        self.vicon_vehicle_local_position.y = msg.pose.position.x
        self.vicon_vehicle_local_position.z = - msg.pose.position.z

        self.vicon_odometry_msg.pose_frame = 0
        self.vicon_odometry_msg.q = [msg.pose.orientation.w,msg.pose.orientation.y,msg.pose.orientation.x,-msg.pose.orientation.z]
        self.vicon_odometry_msg.position = [msg.pose.position.y,msg.pose.position.x,-msg.pose.position.z]

        self.vicon_vehicle_orientation.q = self.vicon_odometry_msg.q  # copying the same orientation for usage with lidar scanner

        self.vehicle_local_position_publisher.publish(self.vicon_vehicle_local_position)
        self.vicon_odometry_publisher.publish(self.vicon_odometry_msg)
        self.vehicle_local_orientation_publisher.publish(self.vicon_vehicle_orientation)

    def vehicle_local_velocity_callback_vicon(self,msg):
        # receiving vehicle attitude from vicon (converting from ENU to NED)
        self.vicon_vehicle_local_position.vx = msg.twist.linear.y
        self.vicon_vehicle_local_position.vy = msg.twist.linear.x
        self.vicon_vehicle_local_position.vz = - msg.twist.linear.z

        self.vicon_odometry_msg.angular_velocity = [msg.twist.angular.y,msg.twist.angular.x,-msg.twist.angular.z]
    
    def vehicle_onboard_odometry_callback(self,msg):
        # receiving vehicle attitude for trying with MPC
        if self.simulation:
            msg_pub = FDOrientationAndRate()
            msg_pub.timestamp = msg.timestamp
            
            vehicle_attitude_quaternion_FRD_Joint = convert_rotation(self.actual_joint_angle,msg.q)
            ######### TEMP TRIAL BELOW, ACTUAL LINE ABOVE ##################
            # vehicle_attitude_quaternion_FRD_Joint = convert_rotation(-self.actual_joint_angle,self.simulation_vehicle_local_orientation.q)
            ################################################################

            rotation_matrix_FRD_Joint = q2rotmat(vehicle_attitude_quaternion_FRD_Joint).copy()
            
            [self.roll,self.pitch,self.yaw] = rotmat2rpy(rotation_matrix_FRD_Joint)
            msg_pub.attitude = [self.roll, self.pitch, self.yaw]


            odometry_angular_velocity_FRD_Pixhawk = np.array([[msg.angular_velocity[0]],[msg.angular_velocity[1]],[msg.angular_velocity[2]]])
            odometry_angular_velocity_FRD_Joint = convert_axes(self.actual_joint_angle,odometry_angular_velocity_FRD_Pixhawk)
            msg_pub.angular_velocity = [float(odometry_angular_velocity_FRD_Joint[0]), float(odometry_angular_velocity_FRD_Joint[1]), float(odometry_angular_velocity_FRD_Joint[2])]

            self.vehicle_local_orientation_and_rate_publisher.publish(msg_pub)

        else:
            pass

    def actual_joint_angle_callback(self,msg):
        self.actual_joint_angle = msg.data

    def simulation_confirmation_callback(self,msg):
        self.simulation = msg.data

    #================================================================= MAIN FUNCTION ========================================================#

def main(args=None):
    rclpy.init(args=args)               # initiating rclpy
    vehicle_local_position_update = VehicleLocalPositionUpdate()  #initializing class
    rclpy.spin(vehicle_local_position_update)         # running in loop

    #ending the processes (currently, the programme does not reach this point)
    outerloop_error_calculations.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
