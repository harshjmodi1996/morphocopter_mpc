
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
from processTrack import *
import numpy as np
import casadi as ca
import csv

'''Global variables'''

linear_scale = 1.0 # scale the  to this number to make the hessian better

# MorphoCopter 3.0 physical parameters
g  = 9.80665      # [m.s^2] gravitational accerelation
m  = 1.8       # [kg] total mass (with Lighthouse deck)
l = 0.228 * linear_scale# scale the  to this number to make the hessian better        # [m] distance from center of mass to motor center
wd = 0.165/2.0*0.0  * linear_scale        # [m] half width of the drone at maximum folding
pl = 0.08  * linear_scale       # [m] propeller radius
delta = 25.0*np.pi/180.0  # [rad] fixed tilt of motors
km = 0.055     # [Nm/(kg*m/s^2)] reaction moment/thrust ratio of the motor-propeller system - calculated from experiments
waypoint_reach_threshold = 0.05  # [m] threshold distance to consider a waypoint reached (useful when MPC trajectory is very close with BIT* reference)

Ixx_u = 0.1625*10**-3  # [kg.m^2] inertia of upper arm about axis aligned with its length and COM of the whole drone
Iyy_u = 7.2470*10**-3  # [kg.m^2] inertia of upper arm about axis perpendicular to its length and COM of the whole drone
Izz_u = 7.1648*10**-3  # [kg.m^2] inertia of upper arm along z axis (vertical axis of the drone)
Ixx_l = 2.7081*10**-3  # [kg.m^2] inertia of lower arm about axis aligned with its length and COM of the whole drone
Iyy_l = 10.8659*10**-3  # [kg.m^2] inertia of lower arm about axis perpendicular to its length and COM of the whole drone
Izz_l = 9.9366*10**-3  # [kg.m^2] inertia of lower arm along z axis (vertical axis of the drone)

# Control constraints
U1_MIN = m*g/8.0
U1_MAX = m*g*2.0
U2_MIN = m*g/8.0
U2_MAX = m*g/2.0
U3_MIN = m*g/8.0
U3_MAX = m*g/2.0
U4_MIN = m*g/8.0
U4_MAX = m*g/2.0
UJ_MIN = -1.8642 # [Nm] minimum joint angle torque - based on hardware limitations (negative of thrust limitation)
UJ_MAX = 1.8642 # [Nm] maximum joint angle torque - based on hardware limitations

U_HOV = m*g/4.0  # Hover thrust
U_REF = np.array([U_HOV, U_HOV, U_HOV, U_HOV, 0])  # Reference control input (hover), 5th is the joint angle torque

# State
n_states = 14

init_zeta = np.array([0.0,0.0,-0.0,                  # x, y, z
                      0.0,0.0,0.0,                  # vx, vy, vz
                      0.0,0.0,0.0,                  # roll, pitch, yaw
                      0.0,0.0,0.0,                  # roll rate (p), pitch rate (q), yaw rate (r)
                      0.0,0.0])                     # joint angle and joint angle velocity
init_zeta[0:6] *= linear_scale  # scale the position and velocity states

# Control
n_controls = 5

control_type = 'MPC+PID'  # 'MPC+PID' or 'MPC+Direct'

# timing parameters
nom_vel = 0.2*linear_scale # [m/s] nominal target velocity of the drone
nom_vel_z = 0.5*linear_scale # [m/s] nominal target vertical velocity of the drone
T_del = 0.1             # time between steps in seconds
N = int(20/0.2*0.5)              # number of shooting nodes
Tf = N * T_del

# Processing Reference Track
track="morphocopter_waypoints.csv"
reference_waypoints = getTrack(track)
upscaled_reference_waypoints = linear_scale * upscale_waypoints_based_on_distance(reference_waypoints, nom_vel/10.0 * T_del / 2.0, log_csv=True, csv_filename="upscaled_waypoints.csv", smoothing_window_size=1)
average_waypoint_distance = compute_average_distance(upscaled_reference_waypoints)
max_waypoint_index = np.shape(upscaled_reference_waypoints)[0]  # total number of waypoints

# Yaw settings
yaw_lowpass_prev_weight = 0.7 # Weight for the previous yaw value in the low pass filter
yaw_lowpass_prev_weight_for_MPC_weights = 0.9 # Weight for the previous yaw value in the low pass filter for calculating weights in MPC

# Joint angle settings
joint_angle_obstacle_consideration_dist = 1.5 * linear_scale  # [m] Minimum distance to obstacles to consider joint_angle_calculations
active_waypoints_horizon_min = 1.0 * linear_scale # [m] min distance to consider waypoints for reference trajectory
active_waypoints_consideration_distance = max(active_waypoints_horizon_min, 1.5 * nom_vel * Tf)
clearance_required_joint_angle = 2.0 * pl # [m] minimum clearance required between propeller and the closest obstacle - used for joint angle calculation
z_obst_consideration = 0.75  * linear_scale # [m] z-height consideration for obstacle avoidance - used for joint angle calculation (useful when LiDAR will be tilted with respect to horizon)

# Obstacle avoidance settings
max_obstacle_segments = 20  # Maximum number of obstacle segments to consider in MPC
obstacle_cost_weight = 1.0 # Penalty multiplier for the cost
obstacle_activation_dist = 0.6  * linear_scale# [m] The distance at which the penalty starts to apply
min_gap_width = 0.1 *linear_scale# [m] Minimum gap width for obstacle avoidance narrow gap reduction

# Scaling terminal shooting node weight
terminal_weight_multiplier_xy = 1.0  # Multiplier for the terminal state weights in MPC compared to stage weights
terminal_weight_multiplier_z = 1.0
terminal_weight_multiplier_yaw = 1.0
terminal_weight_multiplier_joint_angle = 1.0


# Penalties
penalty_long = 0.5333*1.0/linear_scale  # Penalty for longitudinal deviation along the trajectory
penalty_lat = 0.02666*1.0/linear_scale  # Penalty for lateral devation along the trajectory
penalty_z = 4.0/linear_scale  # Penalty for vertical deviation along the trajectory
penalty_yaw = 2.0 # Penalty for yaw deviation from reference    
penalty_J = 20.0 # Penalty for Joint angle deviation from reference

penalty_Ui = 0.05*m*g # Penalty for control input deviation from reference
penalty_UJ = 0.0133*Izz_u # Penalty for joint angle torque deviation from reference
                               
def get_Q_xy(ref_yaw, penalty_long, penalty_lat):
    # Find out Penalty Tensor to distribute longitudinal and lateral penalty acording to reference yaw
    # Pre-calculate sin and cos for efficiency and readability
    c = ca.cos(ref_yaw)
    s = ca.sin(ref_yaw)
    c2 = c**2
    s2 = s**2
    sc = s*c
    # Simplified matrix construction
    Q_xy = np.array([[penalty_long*c2 + penalty_lat*s2,   (penalty_long-penalty_lat)*sc],
                   [(penalty_long-penalty_lat)*sc, penalty_long*s2 + penalty_lat*c2]])
    return Q_xy

# State weights on x, y, z , yaw, joint_angle
ref_yaw = 0.0  # Initial heading of the trajectory in radians
Q_xy = get_Q_xy(ref_yaw, penalty_long, penalty_lat)
Q = np.diag([0.0, 0.0, penalty_z, penalty_yaw, penalty_J])
Q[0:2,0:2] = Q_xy

# Control weights on U1, U2, U3, U4, joint angle torque
R = np.diag([penalty_Ui, penalty_Ui, penalty_Ui, penalty_Ui, penalty_UJ])
 
'''
Explanation of Q_xy formulation:

High-Level Concept: Rotating the Cost Function
Imagine having a simple cost function that penalizes errors along the X and Y axes:
 cost = penalty_x * error_x² + penalty_y * error_y². This corresponds to a diagonal weight matrix Q = [[penalty_x, 0], [0, penalty_y]].

The goal here is to penalize errors along a specific trajectory direction (longitudinal) and perpendicular to it (lateral). 
This is conceptually equivalent to rotating the coordinate system so that its new "X-axis" aligns with your trajectory's yaw angle. 
The matrix Q_xy you've constructed is precisely the result of applying this rotation to the simple diagonal weight matrix.

Mathematical Breakdown
The matrix Q_xy is a result of the formula Q = R' * Q_diag * R, where:
(not to be confused with the R - control weights, this R is just for explanation)

Q_diag is the diagonal matrix of penalties in the trajectory's frame: [[penalty_long, 0], [0, penalty_lat]].
R is the rotation matrix that transforms coordinates from the world frame (x, y) to the trajectory's frame (longitudinal, lateral).
R' is the transpose of R.
Let's verify this with your code. Let c = cos(yaw) and s = sin(yaw). The rotation matrix R is:

R = [[ c, s],
     [-s, c]]

Calculating R' * Q_diag * R gives:

   [[c, -s],   *   [[p_long,    0],   *   [[c, s],
    [s,  c]]        [0,      p_lat]]        [-s, c]]

=  [[c*p_long, -s*p_lat],   *   [[c, s],
    [s*p_long,  c*p_lat]]        [-s, c]]

=  [[c²*p_long + s²*p_lat,   c*s*p_long - s*c*p_lat],
    [s*c*p_long - c*s*p_lat,   s²*p_long + c²*p_lat]]

=  [[p_long*c² + p_lat*s²,   (p_long - p_lat)*s*c],
    [(p_long - p_lat)*s*c,   p_long*s² + p_lat*c²]]
'''

''' Helper functions'''

def euclidean_distance(p1, p2):
    ''' Calculate the Euclidean distance between two points p1 and p2
        p1, p2 : 3D points in the form of numpy arrays or lists
        returns : Euclidean distance as a float'''
    
    return np.linalg.norm(np.array(p1) - np.array(p2))

def find_joint_angle(current_location, vehicle_orientation_rpy, ref_xyz, line_segments, zeta_N):
    """
    Optimized function to find the required joint angle for obstacle avoidance.
    This version uses vectorized NumPy operations to replace nested Python loops
    for significantly faster processing.
    """
    # 1. Setup and Pre-computation
    if line_segments is None or line_segments.shape[2] == 0:
        # print(line_segments)
        return 0.0  # No folding required if no obstacles are present

    yaw = vehicle_orientation_rpy[2]  # Extract yaw from the vehicle orientation

    # Create check points along the drone's intended path
    rear_location = current_location - l * np.array([np.cos(yaw), np.sin(yaw), 0])
    rear_mid_location = current_location - 0.5 * l * np.array([np.cos(yaw), np.sin(yaw), 0])
    
    if zeta_N is not None:
        joint_angle_calc_positions = zeta_N[0:3, :].T  # predicted positions from last OCP solution
    else:
        joint_angle_calc_positions = ref_xyz  # Fallback to reference if no previous solution

    check_points = np.vstack(([rear_location, rear_mid_location, current_location], joint_angle_calc_positions))
    # print(check_points)

    num_check_points = check_points.shape[0]

    # 2. Vectorized Intersection Calculation
    # Expand dimensions for broadcasting: (num_check_points, 1, 2) and (1, num_segments, 2)
    chk_pts_xy = check_points[:, np.newaxis, :2]
    p1_xy = line_segments[0, :2, :].T[np.newaxis, :, :]
    p2_xy = line_segments[1, :2, :].T[np.newaxis, :, :]

    # Movement direction for each check point
    direction_vec = np.diff(check_points, axis=0, append=check_points[-1:, :])
    direction_xy = direction_vec[:, :2]
    norm_direction_xy = np.linalg.norm(direction_xy, axis=1, keepdims=True)
    # Avoid division by zero for stationary points
    direction_xy = np.divide(direction_xy, norm_direction_xy, where=norm_direction_xy > 1e-6, out=np.zeros_like(direction_xy))

    # Perpendicular direction vector for each check point
    perp_xy = np.hstack([-direction_xy[:, 1:2], direction_xy[:, 0:1]])
    perp_xy = perp_xy[:, np.newaxis, :] # Shape: (num_check_points, 1, 2)

    # Solve the linear system Ax=b for all combinations at once
    # A = [perp_xy, -seg_vec_xy], b = p1_xy - chk_pt_xy
    seg_vec_xy = p2_xy - p1_xy
    b = p1_xy - chk_pts_xy

    # Using Cramer's rule for 2x2 systems is faster than np.linalg.solve in a loop
    det_A = perp_xy[:, :, 0] * -seg_vec_xy[:, :, 1] - perp_xy[:, :, 1] * -seg_vec_xy[:, :, 0]

    # Initialize t and s with invalid values (NaN)
    t = np.full_like(det_A, np.nan)
    s = np.full_like(det_A, np.nan)

    # Calculate t and s only where the determinant is non-zero (non-parallel lines)
    valid_det_mask = np.abs(det_A) > 1e-6
    t[valid_det_mask] = (b[:, :, 0] * -seg_vec_xy[:, :, 1] - b[:, :, 1] * -seg_vec_xy[:, :, 0])[valid_det_mask] / det_A[valid_det_mask]
    s[valid_det_mask] = (perp_xy[:, :, 0] * b[:, :, 1] - perp_xy[:, :, 1] * b[:, :, 0])[valid_det_mask] / det_A[valid_det_mask]

    # 3. Filter Valid Intersections
    # Z-height check
    z_min_seg = np.min(line_segments[:, 2, :], axis=0)[np.newaxis, :]
    z_max_seg = np.max(line_segments[:, 2, :], axis=0)[np.newaxis, :]
    chk_pt_z = check_points[:, 2][:, np.newaxis]
    z_valid_mask = (z_max_seg >= chk_pt_z - z_obst_consideration) & (z_min_seg <= chk_pt_z + z_obst_consideration)

    # Intersection must be on the segment (0 <= s <= 1) and within a reasonable range
    s_valid_mask = (s >= 0.0) & (s <= 1.0)
    t_valid_mask = np.abs(t) <= 0.5 # This range seems small, but kept from original logic

    # Combine all masks
    final_mask = z_valid_mask & s_valid_mask & t_valid_mask

    # Apply mask: invalid intersections become NaN
    t_filtered = np.where(final_mask, t, np.nan)

    # 4. Determine Required Joint Angle
    # Find the closest intersection distance from each side for each check point
    min_pos_dist = np.nanmin(np.where(t_filtered >= 0, t_filtered, np.inf), axis=1)
    max_neg_dist = np.nanmax(np.where(t_filtered < 0, t_filtered, -np.inf), axis=1) # abs is applied later

    # Combine distances from both sides
    nearest_dist = np.fmin(min_pos_dist, np.abs(max_neg_dist))

    # Calculate required joint angle only for relevant points
    # Initialize with no folding
    ref_J_temp = np.zeros(num_check_points)
    
    # Identify points close enough to an obstacle to require folding
    needs_folding_mask = (nearest_dist < joint_angle_obstacle_consideration_dist)
    
    if np.any(needs_folding_mask):
        max_half_width = nearest_dist[needs_folding_mask] - clearance_required_joint_angle
        
        # Calculate joint angle based on the available width
        # Ensure argument to arcsin is within [-1, 1]
        arg = np.clip((max_half_width - pl / 2.0) / l, -1.0, 1.0)
        calculated_J = np.pi / 2.0 - 2.0 * np.arcsin(arg)
        
        # Update only the points that need folding, ensuring J is within [0, pi/2]
        ref_J_temp[needs_folding_mask] = np.clip(calculated_J, 0, np.pi / 2.0)

    # The final reference angle is the maximum required angle across all check points
    max_ref_J = np.max(ref_J_temp) if ref_J_temp.size > 0 else 0.0
    return max_ref_J

def update_line_segment_parameters(line_segments, line_segment_capture_location):
    '''Update the obstacle avoidance constraints parameters with the current line segments'''
    if line_segments is not None:
        p_values = np.ones(5 * max_obstacle_segments + 2)*9999.9

        num_actual_segments = line_segments.shape[2]
        
        # Fill parameter values for available segments
        for i in range(min(num_actual_segments, max_obstacle_segments)):
            # p1_x, p1_y
            p_values[5*i]   = line_segments[0, 0, i]
            p_values[5*i+1] = line_segments[0, 1, i]
            # p2_x, p2_y
            p_values[5*i+2] = line_segments[1, 0, i]
            p_values[5*i+3] = line_segments[1, 1, i]

            p_values[5*i+4] = int(line_segments[1, 3, i]/linear_scale)  # cluster_id

        p_values[-2] = line_segment_capture_location[0]  # line_segment_capture_location_x
        p_values[-1] = line_segment_capture_location[1]  # line_segment_capture_location_y
        return p_values

def filter_active_waypoints(upscaled_reference_waypoints, current_waypoint_index, t_prev_run, vehicle_velocity):
    vehicle_velocity_norm = np.linalg.norm(vehicle_velocity[0:2])
    vehicle_velocity_norm_z = np.abs(vehicle_velocity[2])
    # active waypoints stores the waypoints near the current waypoint to reduce computation
    active_waypoints = np.array([[upscaled_reference_waypoints[current_waypoint_index,0], \
                                    upscaled_reference_waypoints[current_waypoint_index,1], \
                                    upscaled_reference_waypoints[current_waypoint_index,2],\
                                        t_prev_run]])
    
    temp_previous_waypoint = upscaled_reference_waypoints[current_waypoint_index, 0:3] 

    waypoint_dist_cum = 0.0
    for n in range(current_waypoint_index+1, max_waypoint_index):
        waypoint_dist = euclidean_distance(upscaled_reference_waypoints[n, 0:2], temp_previous_waypoint[0:2])
        waypoint_dist_z = abs(upscaled_reference_waypoints[n, 2] - temp_previous_waypoint[2])
        waypoint_dist_cum += waypoint_dist

        if waypoint_dist_cum > active_waypoints_consideration_distance:
            break   
        # time = active_waypoints[-1,3] + ((waypoint_dist / min(vehicle_velocity_norm + 0.05*linear_scale, nom_vel))**2.0+\
        #                                     (waypoint_dist_z / min(vehicle_velocity_norm_z + 0.05*linear_scale, nom_vel_z))**2.0)**0.5
        time = active_waypoints[-1,3] + ((waypoint_dist / nom_vel)**2.0+\
                                            (waypoint_dist_z / nom_vel_z)**2.0)**0.5
        # time = active_waypoints[-1,3] + waypoint_dist / nom_vel

        active_waypoints = np.append(active_waypoints, [[upscaled_reference_waypoints[n,0], \
                                                            upscaled_reference_waypoints[n,1], \
                                                            upscaled_reference_waypoints[n,2], \
                                                            time]], axis=0)
        
        temp_previous_waypoint = upscaled_reference_waypoints[n, 0:3]  # update the previous waypoint to the current one
    
    return active_waypoints

def calculate_filtered_yaw(diff_vec, prev_yaw, prev_weight):
    # calculate filtered reference yaw using difference vector and previous yaw
    unfiltered_yaw = np.arctan2(diff_vec[1], diff_vec[0])
    if prev_yaw == None:
        return float(unfiltered_yaw)
    
    ref_yaw = float(prev_weight * prev_yaw + (1 - prev_weight) * unfiltered_yaw)
    return ref_yaw

def log_data(state_steps, control_steps, yref_steps, mpc_iter, t0, e, nx, ny, nu, costs,W_steps, home_path):
        if not state_steps:
            print("No data to log.")
            return
        
        # Stack list of arrays into a single large array
        state_steps_arr = np.stack(state_steps, axis=2)
        control_steps_arr = np.stack(control_steps, axis=2)
        yref_steps_arr = np.stack(yref_steps, axis=2)
        W_steps_arr = np.stack(W_steps, axis=2)

        with open(home_path+'/carl_ws/src/carl_ws_src/morphocopter_planning/acados_mpc/log/exception_log_all_run.csv', 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['Exception occurred:', str(e)])
            writer.writerow(['Last iteration:', mpc_iter])
            writer.writerow(['Last time:', t0,'obstacle activation distance:', obstacle_activation_dist,'obstacle cost weight:', obstacle_cost_weight, 'min gap width:', min_gap_width,'ted',T_del,'linear_scale',linear_scale])
            writer.writerow(['i','x', 'y', 'z', 'vx', 'vy', 'vz', 'phi', 'theta', 'psi', 'p', 'q', 'r', 'J', 'Jdot', 'u1', 'u2', 'u3', 'u4', 'T_J', 'xref', 'yref', 'zref', 'psiref', 'Jref', 'u1ref', 'u2ref', 'u3ref', 'u4ref', 'T_Jref', 'cost','W[0,0]','W[1,1]','W[2,2]','W[3,3]','W[4,4]','R[0,0]','R[1,1]','R[2,2]','R[3,3]','R[4,4]','W[0,1]','W[1,0]'])
            for i in range(mpc_iter):
                for shooting_node in range(N+1):
                    row = [i+1]
                    for j in range(nx):
                        row.append(state_steps_arr[j, shooting_node, i])
                    for j in range(nu):
                        # --- FIX: Extract the applied control (first element of each trajectory) ---
                        if shooting_node != N:
                            row.append(control_steps_arr[j, shooting_node, i])
                        else:
                            row.append(float('nan'))
                    for j in range(ny+nu):
                        row.append(yref_steps_arr[j, shooting_node, i])
                    if i < len(costs):
                        row.append(costs[i])
                    else:
                        row.append(0)
                    
                    for j in range(ny+nu+2):
                        row.append(W_steps_arr[j, shooting_node, i])
                    writer.writerow(row)

