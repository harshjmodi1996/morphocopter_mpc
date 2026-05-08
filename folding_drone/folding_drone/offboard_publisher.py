# This program arms the drone and publishes the offbaord control messages to PX4 for sending offboard commands 
# It subscribes to the topic /folding_drone/in/actuator_commands and uses it to publish the offboard commands

#Importing Necessary libraries
import rclpy
from rclpy.node import Node
import time
import math

#Importing Required Messages
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleControlMode, ActuatorMotors, ActuatorServos, VehicleAttitudeSetpoint, TrajectorySetpoint, VehicleTorqueSetpoint, VehicleThrustSetpoint
from std_msgs.msg import Float32MultiArray, Bool
from folding_drone_msgs.msg import FDActuatorCommands


# Necessary Functions:

# Node class definition

class OffboardPublisher(Node):
    def __init__(self):
        super().__init__('offboard_publisher')  	                    #initiating the node
        
        # Initiating variables
        timer_period = 1.0/50.0                        # seconds - time peried between each message
        self.command_msg =  FDActuatorCommands  #  obsolete comment to be removed: [0]: motor0/--/x [1]: motor1/--//y [2]: motor2/--/z [3]: motor3/thrust_body/yaw [4]: joint_angle [5]: type(motor(0)/attitude(1)/setpoint(2)) qw, qx, qy, qz

        # Initiating required publishers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)  	
        self.actuator_motors_publisher = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', 10)
        self.actuator_servos_publisher = self.create_publisher(ActuatorServos, '/fmu/in/actuator_servos', 10)  # Not working, so using VehicleCommand to control servos  	  
        self.vehicle_attitude_setpoint_publisher = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)   
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.torque_setpoint_publisher = self.create_publisher(VehicleTorqueSetpoint, '/fmu/in/vehicle_torque_setpoint',10)
        self.thrust_setpoint_publisher = self.create_publisher(VehicleThrustSetpoint, '/fmu/in/vehicle_thrust_setpoint',10)

        # Initiating required services
        #self.arming_client=self.create_client(CommandBool,'/mavros/cmd/arming')   

        self.offboard_setpoint_counter = 0
        self.arming_allowed = False # default false until position estimate is received by error_calculation

        # Initiating required subscriptions
        self.pose_subscription = self.create_subscription(FDActuatorCommands,'/folding_drone/in/actuator_commands',self.actuator_commands_callback,10) 
        self.arming_allowed_subscription = self.create_subscription(Bool,'/folding_drone/in/arming_allowed',self.arming_allowed_callback,1) 

        # self.timer = self.create_timer(timer_period, self.timer_callback)  			# creating timer to execute a function at each time_period

    def timer_callback(self):
        if self.offboard_setpoint_counter == 10:  # sending first 10 messages before arming as it is required in PX4
            self.publish_vehicle_command(VehicleCommand().VEHICLE_CMD_DO_SET_MODE,1.0,6.0)
            if self.arming_allowed:
                self.arm()
            else:
                self.offboard_setpoint_counter = 0   # going back to the counter and waiting for arming allowed message
                self.get_logger().info('Position Estimate not Received in last 10 timesteps')
                self.get_logger().info('If running simulation, publish "True" via Bool message on /folding_drone/in/is_simulation')
                self.get_logger().info('------------------------------------------------------------')


        if self.offboard_setpoint_counter < 12:
            self.offboard_setpoint_counter += 1

        self.publish_offboard_control_mode()
        if self.command_msg.direct_actuator:
            self.publish_actuator_motors()
        elif self.command_msg.attitude:
            self.publish_vehicle_attitude()
        elif self.command_msg.position:
            self.publish_trajectory_setpoint()
        elif self.command_msg.thrust_and_torque:
            self.publish_thrust_setpoint()
            self.publsih_torque_setpoint()
        #self.publish_actuator_servos()
        self.publish_actuator()

    def arm(self):
        msg = VehicleCommand()
        msg.command = msg.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0
        self.vehicle_command_publisher.publish(msg)
        self.get_logger().info('Arm Command Sent')

    def disarm(self):
        msg = VehicleCommand()
        msg.command = msg.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 0.0
        self.vehicle_command_publisher.publish(msg)
        self.get_logger().info('Disarm Command Sent')

    def publish_offboard_control_mode(self):
        # telling the PX4 which type of offboard control is used
        msg = OffboardControlMode()
        
        msg.velocity = self.command_msg.velocity
        msg.acceleration = self.command_msg.acceleration
        msg.body_rate = self.command_msg.body_rate
        msg.thrust_and_torque = self.command_msg.thrust_and_torque
        msg.direct_actuator = self.command_msg.direct_actuator
        msg.attitude = self.command_msg.attitude
        msg.position = self.command_msg.position
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_actuator(self):
        # publishing the servo motor control
        msg = VehicleCommand()
        msg.command = msg.VEHICLE_CMD_DO_SET_ACTUATOR
        msg.param1 = self.command_msg.joint_angle/(math.pi/4.0)-1.0
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.vehicle_command_publisher.publish(msg)

    def publish_actuator_motors(self):
        # publishing the main motor controls
        msg = ActuatorMotors()
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        # msg.ACTUATOR_FUNCTION_MOTOR1 = 101
        # msg.NUM_CONTROLS = 4
        msg.control = [0.0]*12
        msg.control[0:4] = self.command_msg.motor_controls
        self.actuator_motors_publisher.publish(msg)

    def publish_actuator_servos(self):
        # for controlling servo motors, but is not working and not using
        msg = ActuatorServos()
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        # msg.NUM_CONTROLS = 1
        msg.control = [0.0]*8
        self.actuator_servos_publisher.publish(msg)

    def publish_vehicle_attitude(self):
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        msg.q_d = self.command_msg.attitude_quaternions
        msg.thrust_body = [0.0,0.0,self.command_msg.thrust_body]
        self.vehicle_attitude_setpoint_publisher.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = self.command_msg.trajectory_setpoint
        msg.yaw = self.command_msg.yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_thrust_setpoint(self):
        msg = VehicleThrustSetpoint()
        msg.xyz = [0.0,0.0,self.command_msg.thrust_body]
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.thrust_setpoint_publisher.publish(msg)

    def publsih_torque_setpoint(self):
        msg = VehicleTorqueSetpoint()
        msg.xyz = self.command_msg.torque_setpoint.copy()
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.torque_setpoint_publisher.publish(msg)

    def actuator_commands_callback(self,msg):
        # storing the received control commands
        self.command_msg = msg
        # print(msg)
        self.timer_callback()

    def arming_allowed_callback(self,msg):
        # checking if arming is allowed (if position estimate received)
        self.arming_allowed = msg.data

    def publish_vehicle_command(self, command, param1, param2):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.vehicle_command_publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)               # initiating rclpy
    offboard_publisher = OffboardPublisher()  #initializing class

    rclpy.spin(offboard_publisher)         # running in loop
    
    #ending the processes (currently, the programme does not reach this point)
    offboard_publisher.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
