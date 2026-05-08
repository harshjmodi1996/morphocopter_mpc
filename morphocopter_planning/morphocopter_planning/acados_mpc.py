#
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#

# reference : "Towards Time-optimal Tunnel-following for Quadrotors", Jon Arrizabalaga et al.

import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Float32, Bool
from folding_drone_msgs.msg import FDOrientationAndRate, FDActuatorCommands

import numpy as np
from time import time
import casadi as ca
import os
import sys
home_path = os.path.expanduser('~')
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(os.path.join(home_path+'/carl_ws/src/carl_ws_src/morphocopter_planning/acados_mpc')))
from common import *  # imports nessary parameters of the morphocopter
from acados_settings import AcadosCustomOcp

sys.path.insert(1,home_path + '/carl_ws/src/carl_ws_src/folding_drone/folding_drone')  # file path for conversion_functions
# import file
from conversion_functions import *  # Functions defining conversion between rotation matrix, quaternion and euler angles


class AcadosMPCNode(Node):
    def __init__(self):
        super().__init__('acados_mpc_node')
        self.get_logger().info('Initializing Acados MPC Node...')

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=10)
        
        self.trajectory_msg = Float64MultiArray()

        self.vehicle_local_position_subscription = self.create_subscription(VehicleLocalPosition,'/folding_drone/out/vehicle_local_position',self.vehicle_local_position_callback, qos_profile=qos_policy)
        self.trajectory_publisher = self.create_publisher(Float64MultiArray, '/folding_drone/in/mpc_trajectory', qos_profile=qos_policy)
        
        self.controls_publisher = self.create_publisher(FDActuatorCommands, '/folding_drone/in/actuator_commands', 10)
        self.arming_allowed_publisher = self.create_publisher(Bool,'/folding_drone/in/arming_allowed',1)

        self.line_segment_capture_location_subscription = self.create_subscription(Float64MultiArray, '/folding_drone/in/obstacle_segment_capture_location', self.line_segment_capture_location_callback, 10)
        self.line_segment_subscription = self.create_subscription(Float64MultiArray,'/folding_drone/in/obstacle_segments',self.line_segment_callback,10)
        self.vehicle_orientation_and_rate_subscription = self.create_subscription(FDOrientationAndRate, '/folding_drone/out/vehicle_orientation_and_rate', self.vehicle_orientation_and_rate_callback, qos_profile=qos_policy)
        self.joint_angle_subscription = self.create_subscription(Float32, '/folding_drone/out/actual_joint_angle', self.joint_angle_callback, qos_profile=qos_policy)
        self.simulation_confirmation = self.create_subscription(Bool,'/folding_drone/in/is_simulation',self.simulation_confirmation_callback, qos_profile=qos_policy)
        
        self.line_segment = np.zeros((2, 3, 0))  # initialize as empty array for line segments generated from lidar data
        self.line_segment_capture_location = np.array([0.0, 0.0, 0.0])  # initialize as zero location for line segment capture location
        self.vehicle_orientation_rpy = [0.0, 0.0, 0.0]  # Initialize vehicle orientation in roll-pitch-yaw format
        self.vehicle_orientation_rates = [0.0, 0.0, 0.0]  # Initialize vehicle orientation rates
        self.simulation = False
        self.vertical_trajectory = False 

        # Initialize Acados OCP
        self.custom_ocp = AcadosCustomOcp()
        self.custom_ocp.setup_acados_ocp()

        # Dimensions
        self.nx = self.custom_ocp.nx
        self.nu = self.custom_ocp.nu
        self.ny = self.custom_ocp.ny

        # Initialize iteration variables
        self.t0 = 0.0
        self.mpc_iter = 0
        self.cost_current = 0.0
        self.current_waypoint_index = 1
        
        # Define states, controls, and slacks as column vectors
        zeta_0 = np.copy(self.custom_ocp.zeta_0)
        self.zeta_N = ca.repmat(np.reshape(zeta_0, (self.nx, 1)), 1, N + 1)

        # Data logging variables
        self.times = []
        self.state_steps = []
        self.control_steps = []
        self.misc_step = []
        self.yref_steps = []
        self.W_steps = []
        self.costs = []

        # Create a timer to run the MPC loop
        self.current_location = np.array([0.0, 0.0, 0.0])  # Initialize current location
        self.vehicle_velocity_direct = np.array([0.0, 0.0, 0.0])  # Initialize current velocity
        self.commanded_joint_angle = 0.0  # Initialize commanded joint angle

        self.timer = self.create_timer(T_del*1.0, self.mpc_loop_callback)

        self.get_logger().info('MPC Node initialized.')
    
    def publish_trajectory(self,N, T_del):
        # message to be sent to the controller contains trajectory x,y,z, vx, vy, vz, roll, pitch, yaw, joint_angle, total thrust, and relative time for each iteration in the nested list format
        # Prepare the trajectory message

        # To align with the state trajectory (N+1 elements), we append a column of zeros.
        u_N_extended = np.hstack((self.custom_ocp.u_N, np.zeros((self.nu, 1))))
        total_thrust_trajectory = np.sum(u_N_extended[:4, :], axis=0, keepdims=True)
        # print(self.custom_ocp.zeta_N)
        trajectory_data = np.vstack((self.custom_ocp.zeta_N[[0,1,2,3,4,5,6,7,8,12],:], 
                        total_thrust_trajectory,
                        np.array([list(range(N+1))])*T_del
                        ))
        trajectory_data[:6] /= linear_scale

        if hasattr(self,'active_waypoints'):
            if self.vertical_trajectory:
                print("Vertical trajectory active")
                trajectory_data[0,:] = self.active_waypoints[0,0] # x
                trajectory_data[1,:] = self.active_waypoints[0,1] # y
                trajectory_data[2,:] = self.active_waypoints[0,2] # z
                trajectory_data[3,:] = 0.0 # vx
                trajectory_data[4,:] = 0.0 # vy
                trajectory_data[5,:] = 0.0 # vz
                trajectory_data[6,:] = 0.0 # roll
                trajectory_data[7,:] = 0.0 # pitch
                trajectory_data[8,:] = 0.0 # yaw
                trajectory_data[9,:] = 0.0 # joint_angle

        # Set layout for the multi-dimensional array
        self.trajectory_msg.layout.dim.clear()
        rows, cols = trajectory_data.shape
        self.trajectory_msg.layout.dim.append(MultiArrayDimension(label='rows', size=rows, stride=rows*cols))
        self.trajectory_msg.layout.dim.append(MultiArrayDimension(label='cols', size=cols, stride=cols))
        self.trajectory_msg.layout.data_offset = 0

        # Flatten the array for Float64MultiArray and convert to list
        self.trajectory_msg.data = trajectory_data.flatten(order='C').tolist()
        self.trajectory_publisher.publish(self.trajectory_msg)

    def publish_controls(self,N, T_del):
        # message to directly send controls to the PX4 wihtout using PID controller
        msg = FDActuatorCommands()

        msg.direct_actuator = True  # Set to True to use direct actuator control
        msg.motor_controls = [float(self.custom_ocp.u_N[0, 0])/(m*g/4.0), float(self.custom_ocp.u_N[1, 0])/(m*g/4.0), float(self.custom_ocp.u_N[2, 0])/(m*g/4.0), float(self.custom_ocp.u_N[3, 0])/(m*g/4.0)]  # First four controls are motor controls
        msg.joint_angle = float(self.custom_ocp.zeta_N[12, 0])  # Joint angle control
        self.controls_publisher.publish(msg)
        self.arming_allowed_publisher.publish(Bool(data=True))  # Publish arming allowed message
    
    def line_segment_callback(self, msg):
        if len(msg.layout.dim) != 3:
            self.get_logger().error("Received line segment data with incorrect number of dimensions.")
            return
    
        num_segments = msg.layout.dim[0].size
        num_points = msg.layout.dim[1].size
        num_coords = msg.layout.dim[2].size

        data = np.array(msg.data, dtype=np.float64)
        transposed_segments = data.reshape((num_segments, num_points, num_coords))

        self.line_segment = np.transpose(transposed_segments, (1, 2, 0))

    def line_segment_capture_location_callback(self, msg):
        if len(msg.data) != 3:
            self.get_logger().error("Received line segment capture location data with incorrect size.")
            return

        self.line_segment_capture_location = np.array(msg.data, dtype=np.float64)

    def mpc_loop_callback(self):
        if self.current_waypoint_index >= max_waypoint_index:
            self.get_logger().info('Track complete! Shutting down.')
            e = 'Track complete'
            self.on_shutdown(e)
            rclpy.shutdown()
            return

        try:
            # Store previous iterate data for plots
            cost = self.custom_ocp.solver.get_cost() if self.mpc_iter > 0 else 0
            self.misc_step.append(np.array([self.t0, cost]))
            self.state_steps.append(self.custom_ocp.zeta_N)
            self.control_steps.append(self.custom_ocp.u_N)
            
            t1 = time()
            # In a real application, this would be updated by a sensor callback

            # Update cost reference
            self.active_waypoints = filter_active_waypoints(upscaled_reference_waypoints, self.current_waypoint_index, (self.mpc_iter-1) * T_del, self.vehicle_velocity_direct*linear_scale)
            
            if self.active_waypoints[1, 0] - self.active_waypoints[0, 0] == 0.0 and \
                self.active_waypoints[2, 0] - self.active_waypoints[1, 0] == 0.0 and \
                self.active_waypoints[1, 1] - self.active_waypoints[0, 1] == 0.0 and \
                self.active_waypoints[2, 1] - self.active_waypoints[1, 1] == 0.0:
                    # to disable MPC trajectory during takeoff/landing phase to help hardware execution
                    self.vertical_trajectory = True
            else:
                self.vertical_trajectory = False
            
            end_condition, yref, W_return = self.custom_ocp.reference_update(self.current_location*linear_scale, self.vehicle_orientation_rpy, self.mpc_iter, 
                self.line_segment*linear_scale, self.line_segment_capture_location*linear_scale, self.active_waypoints)
            
            self.yref_steps.append(yref)
            self.W_steps.append(W_return)

            if end_condition:
                self.get_logger().info("Track complete based on end condition!")
                e = 'Track complete'
                self.on_shutdown(e)
                rclpy.shutdown()
                return

            # Solve the OCP
            if not self.vertical_trajectory: 
                self.custom_ocp.solve_and_sim(self.current_location*linear_scale,self.vehicle_velocity_direct*linear_scale)

            if control_type == 'MPC+PID' and self.custom_ocp.solver.get_status() == 0:
                self.publish_trajectory(N, T_del)  # to use PID controller
            elif control_type == 'MPC+Direct' and self.custom_ocp.solver.get_status() == 0:
                self.publish_controls(N, T_del)   # to directly send controls to the PX4 without using PID controller

            # Update time and iteration
            self.t0 = round(self.t0 + T_del, 3)
            t2 = time()
            ocp_soln_time = t2 - t1
            self.times.append(ocp_soln_time)
            
            # Get new state
            self.zeta_N = self.custom_ocp.zeta_N

            # Advance waypoints:
            for n in range(self.current_waypoint_index, max_waypoint_index):
                # if np.dot(upscaled_reference_waypoints[n+1, 0:3] - upscaled_reference_waypoints[n, 0:3], self.current_location[0:3]*linear_scale - upscaled_reference_waypoints[n, 0:3]) < 0.0:
                if np.linalg.norm(self.current_location*linear_scale - upscaled_reference_waypoints[n, :]) < \
                    np.linalg.norm(self.current_location*linear_scale - upscaled_reference_waypoints[n+1, :])\
                    and np.linalg.norm(self.current_location*linear_scale - upscaled_reference_waypoints[n, :]) > waypoint_reach_threshold:
                        self.current_waypoint_index = n
                        break
                if n == max_waypoint_index - 1:
                    self.current_waypoint_index = n
                

            if self.current_waypoint_index >= max_waypoint_index:
                end_condition =  True
            
            self.mpc_iter += 1

            self.cost_current = self.custom_ocp.solver.get_cost()
            self.get_logger().info(f'MPC Iter {self.mpc_iter} @ t={self.t0}s, Waypoint: {self.current_waypoint_index}, Cost: {round(self.cost_current, 2)}, Progress: {round(self.current_waypoint_index / max_waypoint_index * 100, 2)}%')
            self.get_logger().debug(f'Control: {np.round(self.custom_ocp.u_N[:, 0], 2).T}')
            self.get_logger().debug(f'Sim State: {(np.round(self.zeta_N[:, 0],2).T)}')
            self.costs.append(self.cost_current)

        except KeyboardInterrupt:
            self.get_logger().info('KeyboardInterrupt detected. Exiting the control loop.')
            e = 'KeyboardInterrupt'
            log_data(self.state_steps, self.control_steps, self.yref_steps, self.mpc_iter, self.t0, e, self.nx, self.ny, self.nu, self.costs,self.W_steps, home_path)
            self.on_shutdown(e)
            rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Exception occurred: {e}')
            self.get_logger().info('Exiting the control loop. Saved states, references, and controls up to the last iteration. Check the log folder for details.')
            log_data(self.state_steps, self.control_steps, self.yref_steps, self.mpc_iter, self.t0, e, self.nx, self.ny, self.nu, self.costs,self.W_steps, home_path)
            self.on_shutdown(e)
            rclpy.shutdown()

    def on_shutdown(self, e):
        """
        Cleanup and data logging on node shutdown.
        """
        self.get_logger().info('Node is shutting down, performing final logging and plotting...')
        self.timer.cancel() # Stop the timer

        sqp_max_sec = round(np.array(self.times).max(), 3)
        sqp_avg_sec = round(np.array(self.times).mean(), 3)

        self.get_logger().info(f'Max. solver time\t\t: {sqp_max_sec * 1000} ms')
        self.get_logger().info(f'Avg. solver time\t\t: {sqp_avg_sec * 1000} ms')

        # Log data on successful completion
        log_data(self.state_steps, self.control_steps, self.yref_steps, self.mpc_iter, self.t0, e, self.nx, self.ny, self.nu, self.costs, self.W_steps, home_path)

    def vehicle_local_position_callback(self, msg):
        self.current_location = np.array([msg.x,msg.y,msg.z])
        self.vehicle_velocity_direct = np.array([msg.vx,msg.vy,msg.vz])

    def vehicle_orientation_and_rate_callback(self, msg):
        self.vehicle_orientation_rpy = msg.attitude
        self.vehicle_orientation_rates = msg.angular_velocity

    def joint_angle_callback(self, msg):
        self.commanded_joint_angle = msg.data
    
    def simulation_confirmation_callback(self,msg):
        self.simulation = msg.data

def main(args=None):
    rclpy.init(args=args)
    acados_mpc_node = AcadosMPCNode()
    try:
        rclpy.spin(acados_mpc_node)
    except KeyboardInterrupt:
        acados_mpc_node.get_logger().info('KeyboardInterrupt detected. Shutting down the MPC node and logging data.')
        e = 'KeyboardInterrupt'
        log_data(acados_mpc_node.state_steps, acados_mpc_node.control_steps, acados_mpc_node.yref_steps, acados_mpc_node.mpc_iter, acados_mpc_node.t0, e, acados_mpc_node.nx, acados_mpc_node.ny, acados_mpc_node.nu, acados_mpc_node.costs, acados_mpc_node.W_steps, home_path)
        acados_mpc_node.on_shutdown(e)
        
    acados_mpc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()