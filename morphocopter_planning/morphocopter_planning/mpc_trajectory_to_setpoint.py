import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from folding_drone_msgs.msg import FDSetpoint
import numpy as np
import os
import sys
home_path = os.path.expanduser('~')
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(os.path.join(home_path+'/carl_ws/src/carl_ws_src/morphocopter_planning/acados_mpc')))
from common import *

class TrajectoryFollower(Node):
    # Subscribes to a trajectory published as a Float64MultiArray, converts it, and publishes setpoints based on elapsed time.
    def __init__(self):
        super().__init__('trajectory_follower_node')

        # Parameters
        # self.declare_parameter('publish_frequency', 100.0)
        # publish_freq = self.get_parameter('publish_frequency').get_parameter_value().double_value

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=10)

        # Subscriber to the MPC trajectory (as Float64MultiArray)
        self.trajectory_subscription = self.create_subscription(Float64MultiArray,'/folding_drone/in/mpc_trajectory',self.trajectory_callback,qos_profile=qos_policy)
        
        
        # Publisher for the FDSetpoint
        self.setpoint_publisher = self.create_publisher(FDSetpoint,'/folding_drone/in/setpoint',10)

        self.trajectory_point_counter = 1

        # Timer to publish setpoints at a fixed rate
        self.timer = None  # Initialize timer as None
        # self.trajectory_received = False  # Flag to check if trajectory has been received
        # self.timer = self.create_timer(1.0 / publish_freq, self.publish_setpoint)

        self.trajectory_received = False  # Flag to check if trajectory has been received
        self.timer_rate = 1/100.0  # Timer rate in seconds

        self.get_logger().info('Trajectory Follower Node has been started.')

    def parse_trajectory(self, msg):
        try:
            # Extract dimensions from the layout
            if len(msg.layout.dim) != 2:
                self.get_logger().error(f"Expected 2 dimensions in MultiArray, got {len(msg.layout.dim)}")
                return
            
            # print(msg)
            
            self.rows = msg.layout.dim[0].size
            self.cols = msg.layout.dim[1].size
            
            # Reshape the flat data array into the 2D trajectory data
            self.trajectory_data = np.array(msg.data).reshape((self.rows, self.cols))

            # Expected order: x,y,z, vx,vy,vz, roll,pitch,yaw, joint_angle, total thrust, time
            if self.rows != 12:
                self.get_logger().error(f"Expected 11 rows in trajectory data, got {rows}")
                return

        except Exception as e:
            self.get_logger().error(f"Failed to process trajectory message: {e}")

    def trajectory_callback(self, msg: Float64MultiArray):
        # Receives a Float64MultiArray and stores in self.trajectory_data
        # if self.trajectory_point_counter == N:
        if True:
            if self.trajectory_point_counter >= 1:
                self.trajectory_point_counter = 1 # Counter for trajectory points, resets after receiving a 3 new trajectories
                self.parse_trajectory(msg)      
                self.base_time = self.get_clock().now().nanoseconds*10**(-9)  # Store the time when the trajectory was received

        if not self.trajectory_received:
            
            self.parse_trajectory(msg) 
            self.base_time = self.get_clock().now().nanoseconds*10**(-9)
            self.timer = self.create_timer(self.timer_rate, self.timer_callback)
            self.trajectory_received = True  # Set the flag to True after the first trajectory is received
            self.get_logger().info('Trajectory received and timer started.')

    def timer_callback(self):
        msg = FDSetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        current_time = self.get_clock().now().nanoseconds*10**(-9) - self.base_time  # Calculate elapsed time since trajectory was received
        # print(current_time)
        if current_time > self.trajectory_data[11,self.trajectory_point_counter]-self.trajectory_data[11,self.trajectory_point_counter-1]:
            self.trajectory_point_counter += 1 
            self.base_time = self.get_clock().now().nanoseconds*10**(-9)  # Reset base time for the next point

        self.trajectory_point_counter = min(self.trajectory_point_counter,N-2)  # Ensure i does not exceed the number of trajectory points
        i = self.trajectory_point_counter

        msg.position.x = self.trajectory_data[0, i+2]
        msg.position.y = self.trajectory_data[1, i+2]
        msg.position.z = self.trajectory_data[2, i+2]

        msg.velocity.x = self.trajectory_data[3, i+2]
        msg.velocity.y = self.trajectory_data[4, i+2]
        msg.velocity.z = self.trajectory_data[5, i+2]

        # msg.position.x = self.trajectory_data[0, i-1] + current_time * (self.trajectory_data[0,i]-self.trajectory_data[0,i-1]) / (self.trajectory_data[11,i] - self.trajectory_data[11,i-1] + 1e-6) # Interpolate position
        # msg.position.y = self.trajectory_data[1, i-1] + current_time * (self.trajectory_data[1,i]-self.trajectory_data[1,i-1]) / (self.trajectory_data[11,i] - self.trajectory_data[11,i-1] + 1e-6)
        # msg.position.z = self.trajectory_data[2, i-1] + current_time * (self.trajectory_data[2,i]-self.trajectory_data[2,i-1]) / (self.trajectory_data[11,i] - self.trajectory_data[11,i-1] + 1e-6)
        # # print(msg.position.z)
        # msg.velocity.x = self.trajectory_data[3, i-1] + current_time * (self.trajectory_data[3,i]-self.trajectory_data[3,i-1]) / (self.trajectory_data[11,i] - self.trajectory_data[11,i-1] + 1e-6)
        # msg.velocity.y = self.trajectory_data[4, i-1] + current_time * (self.trajectory_data[4,i]-self.trajectory_data[4,i-1]) / (self.trajectory_data[11,i] - self.trajectory_data[11,i-1] + 1e-6)
        # msg.velocity.z = self.trajectory_data[5, i-1] + current_time * (self.trajectory_data[5,i]-self.trajectory_data[5,i-1]) / (self.trajectory_data[11,i] - self.trajectory_data[11,i-1] + 1e-6)
        # # # I think control inputs (total thrust) can't be used in trajectory output of MPC, because the MPC doesn't vary controls for all the future shooting nodes. So we can't use acceleration command
        T = self.trajectory_data[10, i-1] + current_time * (self.trajectory_data[10,i]-self.trajectory_data[10,i-1])/(self.trajectory_data[11,i] - self.trajectory_data[11,i-1] + 1e-6)*np.cos(delta)  # total thrust in NED convention, delta is the fixed tilt of the propellers in the morphocopter
        roll = self.trajectory_data[6, i-1] + current_time * (self.trajectory_data[6,i]-self.trajectory_data[6,i-1]) / (self.trajectory_data[11,i] - self.trajectory_data[11,i-1] + 1e-6)
        pitch = self.trajectory_data[7, i-1] + current_time * (self.trajectory_data[7,i]-self.trajectory_data[7,i-1]) / (self.trajectory_data[11,i] - self.trajectory_data[11,i-1] + 1e-6)
        yaw = self.trajectory_data[8, i-1] + current_time * (self.trajectory_data[8,i]-self.trajectory_data[8,i-1]) / (self.trajectory_data[11,i] - self.trajectory_data[11,i-1] + 1e-6)
        
        ax_body = -T/m*np.sin(pitch)
        ay_body = T/m*np.sin(roll)
        
        # msg.acceleration.x = ax_body * np.cos(yaw) - ay_body * np.sin(yaw)
        # msg.acceleration.y = ax_body * np.sin(yaw) + ay_body * np.cos(yaw)
        # msg.acceleration.z = -T/m+g

        # yaw_val = self.trajectory_data[8, i-1] + current_time * (self.trajectory_data[8,i]-self.trajectory_data[8,i-1]) / (self.trajectory_data[11,i] - self.trajectory_data[11,i-1] + 1e-6)
        yaw_val = self.trajectory_data[8,i+2]
        msg.yaw = np.clip(yaw_val, -1.57, 1.57)
        msg.joint_angle = self.trajectory_data[9, i+2]
        # print(msg.position.z)

        self.setpoint_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()