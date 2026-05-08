def update_constraints(self, line_segments):
        # Control constraints
        self.ocp.constraints.lbu = np.array([U1_MIN, U2_MIN, U3_MIN, U4_MIN])
        self.ocp.constraints.ubu = np.array([U1_MAX, U2_MAX, U3_MAX, U4_MAX])
        self.ocp.constraints.idxbu = np.array([0, 1, 2, 3])

        # State constraints (joint angle)
        self.ocp.constraints.lbx = np.array([0.0])
        self.ocp.constraints.ubx = np.array([np.pi/2.0])  # Joint angle limits
        self.ocp.constraints.idxbx = np.array([12])

        # Nonlinear obstacle avoidance constraints
        if line_segments is not None and line_segments.shape[2] > 0:
            num_segments = line_segments.shape[2]
            drone_pos = self.ocp.model.x[0:2] # Drone's XY position from state vector
            drone_joint_angle = self.ocp.model.x[12]  # Drone's joint angle from state vector
            drone_roll = self.ocp.model.x[6]  # Drone's roll from state vector
            drone_pitch = self.ocp.model.x[7]  # Drone's pitch from state
            drone_yaw = self.ocp.model.x[8]  # Drone's yaw from state vector
            # R = eul2rotm(drone_roll, drone_pitch, drone_yaw)  # Rotation matrix from roll, pitch, yaw

            # R_x = ca.vertcat(ca.horzcat(1, 0, 0),
            #                  ca.horzcat(0, ca.cos(drone_roll), -ca.sin(drone_roll)),
            #                  ca.horzcat(0, ca.sin(drone_roll), ca.cos(drone_roll)))

            # R_y = ca.vertcat(ca.horzcat(ca.cos(drone_pitch), 0, ca.sin(drone_pitch)),
            #                  ca.horzcat(0, 1, 0),
            #                  ca.horzcat(-ca.sin(drone_pitch), 0, ca.cos(drone_pitch)))

            # R_z = ca.vertcat(ca.horzcat(ca.cos(drone_yaw), -ca.sin(drone_yaw), 0),
            #                  ca.horzcat(ca.sin(drone_yaw), ca.cos(drone_yaw), 0),
            #                  ca.horzcat(0, 0, 1))

            # R = R_x @ R_y @ R_z
            
            # Define the four corner points of the drone in the XY plane symbolically
            # The drone body is modeled as a rectangle that changes shape with the joint angle.
            # The corner positions depend on the drone's position (x,y), orientation (yaw), and joint angle.
            
            # 2D rotation matrix from yaw
            R_yaw = ca.vertcat(
                ca.horzcat(ca.cos(drone_yaw), -ca.sin(drone_yaw)),
                ca.horzcat(ca.sin(drone_yaw),  ca.cos(drone_yaw))
            )

            # Half-width and half-length of the drone's arms projected on XY plane
            half_width = l * ca.sin(np.pi/4.0 - drone_joint_angle/2.0)
            half_length = l * ca.cos(np.pi/4.0 - drone_joint_angle/2.0)

            # Local coordinates of the four corners
            corner1_local = ca.vertcat( half_length,  half_width)
            corner2_local = ca.vertcat(-half_length, -half_width)
            corner3_local = ca.vertcat( half_length, -half_width)
            corner4_local = ca.vertcat(-half_length,  half_width)
            
            # Global coordinates of the four corners
            right_top    = drone_pos + R_yaw @ corner1_local
            left_bottom  = drone_pos + R_yaw @ corner2_local
            left_top     = drone_pos + R_yaw @ corner3_local
            right_bottom = drone_pos + R_yaw @ corner4_local

            drone_corners = [right_top, left_bottom, left_top, right_bottom]

            # print(f"Drone corners: {drone_corners}")
            # print(f"Drone position: {drone_pos}")
            safety_margin = pl*2.0 # Safety distance from obstacles - x times the radius of the propellers

            con_h_exprs = []
            for i in range(num_segments):
                print(num_segments)
                p1 = line_segments[0, 0:2, i]
                p2 = line_segments[1, 0:2, i]
                
                seg_vec = p2 - p1
                seg_len_sq = np.dot(seg_vec, seg_vec)

                for corner_pos in drone_corners:
                    if seg_len_sq < 1e-9: # Segment is a point
                        dist_sq = (corner_pos[0] - p1[0])**2 + (corner_pos[1] - p1[1])**2
                    else:
                        # Project corner position onto the line defined by the segment
                        t = ca.dot(corner_pos - p1, seg_vec) / seg_len_sq
                        
                        # Clamp t to be within [0, 1] to find the closest point on the segment
                        t_clamped = ca.fmax(0.0, ca.fmin(1.0, t))
                        
                        closest_point_on_seg = p1 + t_clamped * seg_vec
                        
                        # Squared distance to the closest point on the segment
                        dist_sq = (corner_pos[0] - closest_point_on_seg[0])**2 + \
                                  (corner_pos[1] - closest_point_on_seg[1])**2

                    # Constraint: dist_sq >= safety_margin^2  (or dist_sq - safety_margin^2 >= 0)
                    con_h_exprs.append(dist_sq - safety_margin**2)

            # Combine all obstacle constraints
            self.ocp.model.con_h_expr = ca.vertcat(*con_h_exprs)
            
            # Set lower bounds for the constraints (h(x) >= 0)
            num_h = self.ocp.model.con_h_expr.shape[0]
            self.ocp.constraints.lh = np.zeros(num_h)
            self.ocp.constraints.uh = np.full(num_h, ACADOS_INFTY)
            # print(self.ocp.constraints.lh)
            # print(self.ocp.constraints.uh)
            # print("Number of constraints:", num_h)
        else:
            # If no obstacles, clear the constraints
            self.ocp.model.con_h_expr = None
            self.ocp.constraints.lh = np.array([])
            self.ocp.constraints.uh = np.array([])