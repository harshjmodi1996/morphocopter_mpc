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

import casadi as ca
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver, AcadosSimSolver, ACADOS_INFTY
from pyrsistent import ny
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
import csv

from common import *
from sys_dynamics import SysDyn

class AcadosCustomOcp:

    def __init__(self):
        self.nx = 0
        self.nu = 0
        self.ny = 0
        self.ns = 0

        self.ocp = None,
        self.solver = None,
        self.integrator = None
        self.sysModel = None

        self.zeta_0 = None
        self.zeta_N = None
        self.u_N = None

        # # Log Hessian for all shooting nodes
        # self.hessian_log_path = 'log/hessian_log.csv'
        # with open(self.hessian_log_path, 'w') as csvfile:
        #     csv_writer = csv.writer(csvfile)
        #     header = ['mpc_iter', 'shooting_node']
        #     csv_writer.writerow(header)

    def setup_acados_ocp(self):
        '''Formulate acados OCP'''

        # create casadi symbolic expressions
        sysModel = SysDyn()
        self.sysModel = sysModel

        zeta_f, dyn_f, u, dyn_fn = sysModel.SetupOde()
        self.zeta_0 = np.copy(init_zeta)

        # create Acados model
        ocp = AcadosOcp()
        model_ac = AcadosModel()
        model_ac.f_expl_expr = dyn_f
        model_ac.x = zeta_f
        model_ac.u = u
        model_ac.name = "morphocopter_model"
        ocp.model = model_ac

        # set dimensions
        ocp.solver_options.N_horizon = N
        self.nx = model_ac.x.size()[0]
        self.nu = model_ac.u.size()[0]
        self.ny = 5 # number of outputs

        self.zeta_N = ca.repmat(np.reshape(self.zeta_0, (self.nx,1)), 1, N+1)
        self.u_N = ca.repmat(U_REF, 1, N)

        # Continuity constraints
        # Note: This initializes the constraint structure. The value is updated iteratively in solve_and_sim via solve_for_x0.
        ocp.constraints.x0 = self.zeta_0

        # Control constraints
        ocp.constraints.lbu = np.array([U1_MIN, U2_MIN, U3_MIN, U4_MIN])
        ocp.constraints.ubu = np.array([U1_MAX, U2_MAX, U3_MAX, U4_MAX])
        ocp.constraints.idxbu = np.array([0, 1, 2, 3])

        # State constraints (joint angle)
        ocp.constraints.lbx = np.array([0.0])
        ocp.constraints.ubx = np.array([np.pi/2.0])  # Joint angle limits
        ocp.constraints.idxbx = np.array([12])

        # ==== Parameters (updated in each iteration) ====
        self.num_obstacle_segments = max_obstacle_segments
        p_obs_no = 5 * self.num_obstacle_segments + 2
        p_obs = ca.MX.sym('p_obs', p_obs_no)

        p_yref_no = self.ny + self.nu  # 5 states + 5 controls
        p_yref = ca.MX.sym('p_yref', p_yref_no)  # paramters to pass reference trajectory and controls

        p_W_no = self.ny + 2 + self.nu
        p_W = ca.MX.sym('p_W', p_W_no)   # parameters to pass cost weights (ny+2 is for setting up long+lat costs)

        ocp.model.p = ca.vertcat(p_obs, p_yref, p_W)  # Concatenate all parameters
        ocp.parameter_values = np.zeros(p_obs_no + p_yref_no + p_W_no) # Initialize with zeros

        # ==== Stage costs ====:
        ocp.cost.cost_type = "EXTERNAL"
        y_expr = ca.vertcat(model_ac.x[0:3], model_ac.x[8], model_ac.x[12], model_ac.u) # Define the expression for the outputs to be tracked
        y_ref_error = y_expr - p_yref  # Calculate the error against the symbolic reference parameter
        W = ca.MX.zeros(self.ny+self.nu,self.ny+self.nu)  # Initialize a zero matrix

        for i in range(self.ny+self.nu):
            W[i,i] = p_W[i]  # Fill the diagonal with the first ny+nu parameters

        W[0,1] = p_W[self.ny+self.nu]
        W[1,0] = p_W[self.ny+self.nu+1]
        tracking_cost_expr = 0.5 * (y_ref_error.T @ W @ y_ref_error) # The external cost is the quadratic form for a single, generic stage

        # ==== Terminal Cost ====:
        ocp.cost.cost_type_e = "EXTERNAL"
        y_expr_e = ca.vertcat(model_ac.x[0:3], model_ac.x[8], model_ac.x[12]) # Terminal expression (states only, no controls)
        y_ref_error_e = y_expr_e - p_yref[0:5] # Terminal reference from parameters (using first 5 elements of p_yref)
        W_e = W[0:5, 0:5]
        tracking_cost_expr_e = 0.5 * (y_ref_error_e.T @ W_e @ y_ref_error_e)

        # ==== Obstacle Avoidance Cost ====
        # The state variables used here are symbolic and will be evaluated at each node.
        drone_pos = ocp.model.x[0:2] # Drone's XY position from state vector
        check_pos = drone_pos
        total_obstacle_cost = 0.0

        closest_points = []  # for obstacle cost
        distances_sq = []   # for obstacle cost
        
        p1 = p_obs[5*i : 5*i+2]
        p2 = p_obs[5*i+2 : 5*i+4]
        cluster_id = p_obs[5*i+4]  

        seg_vec = p2 - p1
        seg_len_sq = ca.dot(seg_vec, seg_vec)

        for i in range(self.num_obstacle_segments):
            p1 = p_obs[5*i : 5*i+2]
            p2 = p_obs[5*i+2 : 5*i+4]
            cluster_id = p_obs[5*i+4]  

            # capture_location = [p_obs[-2], p_obs[-1]]

            seg_vec = p2 - p1
            seg_len_sq = ca.dot(seg_vec, seg_vec)

            # Project drone_pos onto the line defined by the segment
            t = ca.dot(check_pos - p1, seg_vec) / (seg_len_sq + 1e-9) # Add epsilon for stability

            # Clamp t to be within [0, 1] to find the closest point on the segment
            t_clamped = ca.fmax(0.0, ca.fmin(1.0, t))

            closest_point_on_seg = p1 + t_clamped * seg_vec

            # Squared distance to the closest point on the segment
            dist_sq = (check_pos[0] - closest_point_on_seg[0])**2 + \
                (check_pos[1] - closest_point_on_seg[1])**2
            
            distances_sq.append(dist_sq)
            closest_points.append(ca.vertcat(closest_point_on_seg, cluster_id))
            
        # # Find the minimum distance and the corresponding closest point
        min_dist_sq = distances_sq[0]
        closest_point_min_dist = closest_points[0]
        for i in range(self.num_obstacle_segments):
            closest_point_min_dist = ca.if_else(distances_sq[i] < min_dist_sq, closest_points[i], closest_point_min_dist)
            min_dist_sq = ca.fmin(min_dist_sq, distances_sq[i])

        # Calculate passage_scaling_min
        passage_scaling_min = 1.0

        for i in range(self.num_obstacle_segments):
            # find_cancelled_component logic
            obst_location_1 = closest_point_min_dist
            obst_location_2_temp = closest_points[i]

            obst_location_2 = ca.if_else(closest_points[i][0] > 1000.0, obst_location_1, obst_location_2_temp) # dummy segment check
            
            passage_direction = (obst_location_1[:2] - obst_location_2[:2])
            obst_vec_1 = obst_location_1[:2] - drone_pos
            obst_vec_2 = obst_location_2[:2] - drone_pos

            obst_vec_component_1 = (obst_vec_1[0]*passage_direction[0]+obst_vec_1[1]*passage_direction[1])
            obst_vec_component_2 = (obst_vec_2[0]*passage_direction[0]+obst_vec_2[1]*passage_direction[1])

            passage_scaling = ca.fabs(obst_vec_component_1 + obst_vec_component_2) / (ca.fabs(obst_vec_component_1) + ca.fabs(obst_vec_component_2) + 1e-6)

            current_passage_scaling_avoid_same_cluster = ca.if_else(ca.fabs(obst_location_1[2]-obst_location_2[2])<1e-1, 1.0, passage_scaling)
            passage_scaling_min = ca.fmin(passage_scaling_min, current_passage_scaling_avoid_same_cluster)

            ###################################### TEMP #######################################
            # passage_scaling_min = 1.0  # TEMPORARY OVERRIDE TO DISABLE PASSAGE SCALING EFFECT
            ###################################################################################

        # Final obstacle cost calculation
        total_obstacle_cost = obstacle_cost_weight * (1.0-(passage_scaling_min**2.0-1.0)**2.0) * 1/2.7183 * 2.7183**((obstacle_activation_dist**2 - min_dist_sq) / obstacle_activation_dist**2)

        # Add the tracking cost and the obstacle cost together
        ocp.model.cost_expr_ext_cost = tracking_cost_expr + total_obstacle_cost
        ocp.model.cost_expr_ext_cost_e = tracking_cost_expr_e

        # configure itegrator and QP solver
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.tf = Tf
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 1

        ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM" 
        #PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM, PARTIAL_CONDENSING_QPDUNES, PARTIAL_CONDENSING_OSQP, FULL_CONDENSING_DAQP.

        ocp.solver_options.hessian_approx =  "EXACT"  #"GAUSS_NEWTON",  changed to EXACT from GAUSS_NEWTON as the cost is now not standard least square cost (because of obstacle cost)
        # ocp.solver_options.cost_discretization ="INTEGRATOR"
        # ocp.solver_options.qp_solver_cond_N = int(N/2)
        ocp.solver_options.nlp_solver_type = "SQP"
        ocp.solver_options.tol = 1e-3

        ocp.solver_options.nlp_solver_max_iter = 50 #     NLP solver maximum number of iterations. Type: int >= 0 Default: 100

        ## REGULARIZATION:
        ocp.solver_options.regularize_method = 'MIRROR'  
        # MIRROR: performs eigenvalue decomposition H = V^T D V and sets D_ii = max(eps, abs(D_ii))
        # PROJECT: performs eigenvalue decomposition H = V^T D V and sets D_ii = max(eps, D_ii)
        # CONVEXIFY: lgorithm 6 from Verschueren2017, https://cdn.syscop.de/publications/Verschueren2017.pdf, experimental, might not be correct if inequality constraints are active.
        # PROJECT_REDUC_HESS: experimental, should make sure that the reduced Hessian is positive definite. Has to be used with qp_solver_ric_alg = 0 and qp_solver_cond_ric_alg = 0
        # GERSHGORIN_LEVENBERG_MARQUARDT: estimates the smallest eigenvalue of each Hessian block using Gershgorin circles and adds multiple of identity to each block, such that smallest eigenvalue after regularization is at least reg_epsilon
        # DEFAULT: NO_REGULARIZE

        ocp.solver_options.reg_epsilon = 1e-3 # Type: float. Default: 1e-4; Epsilon for regularization, used if regularize_method in [‘PROJECT’, ‘MIRROR’, ‘CONVEXIFY’, ‘GERSHGORIN_LEVENBERG_MARQUARDT’].
        # convex narrow gap working with 1e-3 almost everytime (except when it oscillates about pitch), but failing in narrow gap with 1e-4
        ocp.solver_options.reg_adaptive_eps = False #  Type: bool Default: False; Determines if epsilon is chosen adaptively in regularization used if regularize_method in [‘PROJECT’, ‘MIRROR’], If true, epsilon is chosen block-wise based on reg_max_cond_block. Otherwise, epsilon is chosen globally based on reg_epsilon,
        ocp.solver_options.reg_max_cond_block = 1e7 # Type: float Default: 1e7; Maximum condition number of each Hessian block after regularization with regularize_method in [‘PROJECT’, ‘MIRROR’] and reg_adaptive_eps = True
        ocp.solver_options.reg_min_epsilon = 1e-8 # Type: float Default: 1e-8; Minimum value for epsilon if regularize_method in [‘PROJECT’, ‘MIRROR’] is used with reg_adaptive_eps.
        ocp.solver_options.levenberg_marquardt = 1e-4 # Factor for LM regularization. Type: float >= 0 Default: 0.0.

        # create solver
        self.ocp = ocp
        self.solver = AcadosOcpSolver(ocp, json_file = "planner_ocp.json", generate=False, build=False)  # if not running for the first time after change, make it false
        self.integrator = AcadosSimSolver(ocp)
    
        return True

    def reference_update(self, current_location, vehicle_orientation_rpy, mpc_iter, line_segments, line_segment_capture_location, active_waypoints):
        end_condition = False
        
        t = np.linspace(mpc_iter*T_del, (mpc_iter+N+1-1) * T_del, N+1)
        # print(active_waypoints)

        ref_xyz = np.zeros((N+1, 3))    # Reference position in 3D space

        assigned_any_reference = False
        for i in range(N+1):
            # Interpolate the reference waypoints to get the reference state at each time step
            for j in range(np.shape(active_waypoints)[0]):
                if t[i] < active_waypoints[j, 3]:
                    assigned_any_reference = True
                    waypoint_prev = active_waypoints[j - 1, 0:3]
                    waypoint_next = active_waypoints[j, 0:3]
                    t_prev = active_waypoints[j - 1, 3]
                    t_next = active_waypoints[j, 3]
                    velocity = (waypoint_next - waypoint_prev) / (t_next - t_prev)
                    ref_xyz[i] = waypoint_prev + velocity * (t[i] - t_prev)
                    break
        if not assigned_any_reference:
            end_condition =  True
            ref_xyz[0, 0:3] = current_location[0:3]  # If not reference points were assigned at the end of the path, use the current location as the reference point

        joint_angle_ref = find_joint_angle(current_location, vehicle_orientation_rpy, active_waypoints[:,0:3], line_segments, self.zeta_N)
        print(f'Joint Angle Reference: {joint_angle_ref}')
        
        yref_return = np.zeros((self.ny + self.nu, N+1))
        W_return = np.zeros((self.ny + self.nu + 2, N+1))
        p_obs = update_line_segment_parameters(line_segments, line_segment_capture_location)

        for i in range(N+1):
            next_idx = min(i+1, N)
            # Determine reference point, handling the end of the path
            if not assigned_any_reference:
                ref_xyz_i = current_location[0:3]
            else:
                # Ensure reference doesn't become zero at the end
                if np.all(ref_xyz[i] == 0.0) and i > 0:
                    ref_xyz[i] = ref_xyz[i-1]
                    end_condition = True   
                ref_xyz_i = ref_xyz[i]

            # Calculate yaw reference for trajectory using previous solution
            diff_vec_MPC_weights = (ref_xyz[next_idx] - ref_xyz[next_idx-1])
            diff_vec_prev_sol = (self.zeta_N[0:2, next_idx] - self.zeta_N[0:2, next_idx-1]) if self.zeta_N is not None else diff_vec_MPC_weights
            ref_yaw_i = calculate_filtered_yaw(diff_vec_prev_sol, getattr(self, 'prev_yaw', None), yaw_lowpass_prev_weight)
            self.prev_yaw = ref_yaw_i

            # Assemble the reference vector
            yref = np.array([ref_xyz_i[0], ref_xyz_i[1], ref_xyz_i[2], ref_yaw_i, joint_angle_ref, U_HOV, U_HOV, U_HOV, U_HOV, 0.0])

            # Adjusting weight of the cost function based on the trajectory heading (to have different weights in longitudinal and lateral directions)
            ref_yaw_i_for_MPC_weights = calculate_filtered_yaw(diff_vec_MPC_weights, getattr(self, 'prev_yaw_for_MPC_weights', None), yaw_lowpass_prev_weight_for_MPC_weights)
            self.prev_yaw_for_MPC_weights = ref_yaw_i_for_MPC_weights

            Q_xy_i = get_Q_xy(ref_yaw_i_for_MPC_weights, penalty_long, penalty_lat)
            Q_i = np.copy(Q)
            Q_i[0:2,0:2] = Q_xy_i

            if i == N:
                W = np.array([Q_i[0,0]*terminal_weight_multiplier_xy,Q_i[1,1]*terminal_weight_multiplier_xy,Q_i[2,2]*terminal_weight_multiplier_z,Q_i[3,3]*terminal_weight_multiplier_yaw,Q_i[4,4]*terminal_weight_multiplier_joint_angle, R[0,0],R[1,1],R[2,2],R[3,3],R[4,4],Q_i[0,1]*terminal_weight_multiplier_xy,Q_i[1,0]*terminal_weight_multiplier_xy])
            else:
                W = np.array([Q_i[0,0],Q_i[1,1],Q_i[2,2],Q_i[3,3],Q_i[4,4], R[0,0],R[1,1],R[2,2],R[3,3],R[4,4],Q_i[0,1],Q_i[1,0]])

            # 4. Concatenate all parameters for this stage and set them
            p_values_for_stage_i = np.concatenate([p_obs, yref, W])
            self.solver.set(i, "p", p_values_for_stage_i)

            W_return[:,i] = W
            yref_return[:, i] = yref

        return end_condition, yref_return, W_return

    def solve_and_sim(self, current_location, velocity):
        '''Solve the OCP with multiple shooting, and forward simulate with RK4'''

        print('Solve and Sim Running')
        
        self.zeta_0[0] = current_location[0]  # Update the initial state with the current location from Gazebo (position feedback)
        self.zeta_0[1] = current_location[1]
        self.zeta_0[2] = current_location[2]
        ####################### TEMP #######################
        # self.zeta_0[2] = -0.5 * linear_scale
        ####################################################
        self.zeta_0[3] = velocity[0]
        self.zeta_0[4] = velocity[1]
        # self.zeta_0[5] = velocity[2]

        # Integrate ODE model to get CL estimate (no measurement noise)
        u_0 = self.solver.solve_for_x0(self.zeta_0, fail_on_nonzero_status=True)
        
        zeta_0_sim = self.integrator.simulate(x=self.zeta_0, u=u_0)

        if np.any(np.isnan(zeta_0_sim)):
            # to avoid non-zero solution causing issues in future iterations
            if hasattr(self, 'zeta_0_prev'):
                self.zeta_0 = self.zeta_0_prev.copy()
        else:
            self.zeta_0 = zeta_0_sim

        self.zeta_0_prev = self.zeta_0.copy()

        self.zeta_N = np.array([self.solver.get(i, "x") for i in range(N + 1)]).T
        self.u_N = np.array([self.solver.get(i, "u") for i in range(N)]).T

        # with open(self.hessian_log_path, 'a') as csvfile:
        #     csv_writer = csv.writer(csvfile)
        #     # Loop through all shooting nodes for their hessians
        #     for i in range(0,N):
        #         hessian_block = self.solver.get_hessian_block(i)
        #         log_data = [mpc_iter, i]
        #         for j in range(len(hessian_block)):
        #             log_data += hessian_block[j].tolist()
        #         csv_writer.writerow(log_data)

    def get_cost(self):
        cost = self.solver.get_cost()
        return cost