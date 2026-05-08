# This program creates time based setpoints from provided waypoints

#Importing Necessary libraries
import rclpy
from rclpy.node import Node
import time
import numpy as np
import math

#Importing Required Messages
from folding_drone_msgs.msg import FDWaypoint, FDSetpoint
from geometry_msgs.msg import Point

# Node class definition
class WaypointstoSetpoints(Node):
    def __init__(self):
        super().__init__('waypoints_to_setpoints')  	                    #initiating the node
        
        # Initiating 0 condition variables: 

        timer_period = 1.0/200.0                        # seconds - time peried between each message
        self.wpi = 0 # starting with initial waypoint, tracking which waypoint time has been passed
        self.trajectory_initiated = False # whenver first set of waypoints are received, the setpoint publication is initiated

        self.position = Point(x=0.0,y=0.0,z=0.0)
        self.velocity = Point(x=0.0,y=0.0,z=0.0)
        self.acceleration = Point(x=0.0,y=0.0,z=0.0)
        self.joint_angle = 0.0
        self.yaw = 0.0
        self.initial_velocities = [0.0,0.0,0.0]
        self.final_velocities = [0.0,0.0,0.0]

        # Initiating required publishers
        self.setpoints_publisher = self.create_publisher(FDSetpoint, '/folding_drone/in/setpoint', 10)  
        # Initiating required subscriptions
        self.vehicle_attitude_subscription = self.create_subscription(FDWaypoint,'/folding_drone/in/waypoints',self.waypoints_callback, 10)

        self.timer = self.create_timer(timer_period, self.timer_callback)  			# creating timer to execute a function at each time_period

    #================================================================= TIMER CALLBACK ===========================================================#

    def timer_callback(self):
        self.calculate_setpoint()
        msg = FDSetpoint()
        msg.position = self.position
        msg.velocity = self.velocity
        msg.acceleration = self.acceleration
        msg.joint_angle = self.joint_angle
        msg.yaw = self.yaw
        #print(msg)
        self.setpoints_publisher.publish(msg)

    def calculate_setpoint(self):
        if self.trajectory_initiated:
            self.current_time = self.get_clock().now().nanoseconds*10**(-9) - self.init_time + self.times[0]
            # print(self.current_time)
            

            if self.wpi<len(self.waypoint_positions):
                if self.current_time > self.times[self.wpi]:
                    self.wpi+=1 # advancing the waypoints

            # print(self.wpi)

            if self.wpi<len(self.waypoint_positions):
                self.position = Point()
                self.velocity = Point()
                self.acceleration = Point()

                self.position.x = np.dot(self.coeff[6*(self.wpi-1):6*(self.wpi-1)+6,0],self.time_powers(self.current_time,0))
                self.position.y = np.dot(self.coeff[6*(self.wpi-1):6*(self.wpi-1)+6,1],self.time_powers(self.current_time,0))
                self.position.z = np.dot(self.coeff[6*(self.wpi-1):6*(self.wpi-1)+6,2],self.time_powers(self.current_time,0))
                
                self.velocity.x = np.dot(self.coeff[6*(self.wpi-1):6*(self.wpi-1)+6,0],self.time_powers(self.current_time,1))
                self.velocity.y = np.dot(self.coeff[6*(self.wpi-1):6*(self.wpi-1)+6,1],self.time_powers(self.current_time,1))
                self.velocity.z = np.dot(self.coeff[6*(self.wpi-1):6*(self.wpi-1)+6,2],self.time_powers(self.current_time,1))

                self.acceleration.x = np.dot(self.coeff[6*(self.wpi-1):6*(self.wpi-1)+6,0],self.time_powers(self.current_time,2))
                self.acceleration.y = np.dot(self.coeff[6*(self.wpi-1):6*(self.wpi-1)+6,1],self.time_powers(self.current_time,2))
                self.acceleration.z = np.dot(self.coeff[6*(self.wpi-1):6*(self.wpi-1)+6,2],self.time_powers(self.current_time,2))

                self.joint_angle = self.waypoint_joint_angles[self.wpi]
                # if self.velocity.y !=0.0 and self.velocity.x!=0.0:
                #     self.yaw = math.atan2(self.velocity.y,self.velocity.x)
                # else:
                self.yaw = self.waypoint_yaws[-1]

                # print(self.coeff)


    def time_powers(self,time,derivative):
        if derivative == 0:
            return(np.array([time**0.0,time**1.0,time**2.0,time**3.0,time**4.0,time**5.0]))
        elif derivative == 1:
            return(np.array([0.0,time**0.0,2.0*time**1.0,3.0*time**2.0,4.0*time**3.0,5.0*time**4.0]))
        elif derivative == 2:
            return(np.array([0.0,0.0,2.0*time**0.0,6.0*time**1.0,12.0*time**2.0,20.0*time**3.0]))
        elif derivative == 3:
            return(np.array([0.0,0.0,0.0,6.0*time**0.0,24.0*time**1.0,60.0*time**2.0]))
        elif derivative == 4:
            return(np.array([0.0,0.0,0.0,0.0,24.0*time**0.0,120.0*time**1.0]))

    def spline_interpolation(self):
        np.set_printoptions(suppress=True)
        np.set_printoptions(linewidth=np.inf)
        n = len(self.waypoint_positions)  # number of waypoints including current position
        M = np.zeros((6*(n-1),6*(n-1)))  
        constraints = np.zeros((6*(n-1),3))

        for wpi in range(n-1):
            # filling up initial positions
            M[6*wpi,6*wpi:6*(wpi+1)] = self.time_powers(self.times[wpi],0)
            constraints[6*wpi,:] = np.array([self.waypoint_positions[wpi].position.x,self.waypoint_positions[wpi].position.y,self.waypoint_positions[wpi].position.z])

        for wpi in range(1,n):
            # filling up final positions
            M[6*wpi-3,6*(wpi-1):6*(wpi)] = self.time_powers(self.times[wpi],0)
            constraints[6*wpi-3,:] = np.array([self.waypoint_positions[wpi].position.x,self.waypoint_positions[wpi].position.y,self.waypoint_positions[wpi].position.z])

        # defined initial velocities
        M[1,0:6] = self.time_powers(self.times[0],1)
        constraints[1,:] = np.array(self.initial_velocities)

        # defined final velocities
        M[6*(n-2)+4,6*(n-2):6*(n-1)] = self.time_powers(self.times[n-1],1)
        constraints[6*(n-2)+4,:] = np.array(self.final_velocities)

        # zero initial acceleration
        M[2,0:6] = self.time_powers(self.times[0],2)
        constraints[2,:] = np.array([0.0,0.0,0.0])

        # zero final acceleration
        M[6*(n-2)+5,6*(n-2):6*(n-1)] = self.time_powers(self.times[n-1],2)
        constraints[6*(n-2)+5,:] = np.array([0.0,0.0,0.0])

        for wpi in range(n-2):
            # velocity continuity:
            M[6*wpi+4,6*wpi:6*(wpi+1)]=self.time_powers(self.times[wpi+1],1)
            M[6*wpi+4,6*(wpi+1):6*(wpi+2)]=-self.time_powers(self.times[wpi+1],1)

            # acceleration continuity:
            M[6*wpi+5,6*wpi:6*(wpi+1)]=self.time_powers(self.times[wpi+1],2)
            M[6*wpi+5,6*(wpi+1):6*(wpi+2)]=-self.time_powers(self.times[wpi+1],2)

            # 3rd derivative continuity:
            M[6*wpi+7,6*wpi:6*(wpi+1)]=self.time_powers(self.times[wpi+1],3)
            M[6*wpi+7,6*(wpi+1):6*(wpi+2)]=-self.time_powers(self.times[wpi+1],3)

            # 4th derivative continuity:
            M[6*wpi+8,6*wpi:6*(wpi+1)]=self.time_powers(self.times[wpi+1],4)
            M[6*wpi+8,6*(wpi+1):6*(wpi+2)]=-self.time_powers(self.times[wpi+1],4)


        # print(M)
        # print(constraints)

        self.coeff = np.linalg.solve(M,constraints)
        # print(self.coeff)

        self.trajectory_initiated = True ## first ever trajectory has been generated for publication

    def waypoints_callback(self,msg):
        # receiving vehicle attitude
        self.wpi = 0
        self.waypoint_positions = msg.positions.poses
        self.initial_velocities = msg.initial_velocities
        self.final_velocities = msg.final_velocities
        self.waypoint_yaws = msg.yaws
        self.waypoint_joint_angles = msg.joint_angles
        self.times = msg.times
        self.init_time = self.get_clock().now().nanoseconds*10**(-9)
        self.spline_interpolation()
        
def main(args=None):
    rclpy.init(args=args)               # initiating rclpy
    waypoints_to_setpoints = WaypointstoSetpoints()  #initializing class
    rclpy.spin(waypoints_to_setpoints)         # running in loop

    #ending the processes (currently, the programme does not reach this point)
    waypoints_to_setpoints.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()