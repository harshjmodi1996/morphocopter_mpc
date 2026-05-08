import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import os
import sys
import csv
from folding_drone_msgs.msg import FDOrientationNED
from std_msgs.msg import Float32
from px4_msgs.msg import VehicleLocalPosition
home_path = os.path.expanduser('~')
sys.path.insert(1,home_path + '/carl_ws/src/carl_ws_src/folding_drone/folding_drone')  # file path for conversion_functions
sys.path.append(os.path.abspath(os.path.join(home_path+'/carl_ws/src/carl_ws_src/morphocopter_planning/acados_mpc')))
from common import *
from conversion_functions import q2rotmat, euclidean_distance

from sklearn.cluster import DBSCAN
from sklearn.linear_model import LinearRegression
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

def quaternion_multiply(q1, q2):
    """
    Multiplies two quaternions.

    Args:
        q1 (np.ndarray or list): The first quaternion [w1, x1, y1, z1].
        q2 (np.ndarray or list): The second quaternion [w2, x2, y2, z2].

    Returns:
        np.ndarray: The resulting quaternion [w_res, x_res, y_res, z_res].
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w_res = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x_res = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y_res = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z_res = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return np.array([w_res, x_res, y_res, z_res])


class ScanSubscriber(Node):

    def __init__(self):
        super().__init__('scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_final_simulation',
            self.listener_callback_simulation,
            10)
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_final_hardware',
            self.listener_callback_hardware,
            10)
        # self.subscription = self.create_subscription(
        #     LaserScan,
        #     '/model/folding_drone/lidar',
        #     self.listener_callback,
        #     10)
        
        
        self.segment_publisher = self.create_publisher(Float64MultiArray, 'folding_drone/in/obstacle_segments', 10)
        self.segment_capture_location_publisher = self.create_publisher(Float64MultiArray, 'folding_drone/in/obstacle_segment_capture_location', 10)
        
        self.lidar_offset_z = 0.0 # m (NED convention)
        self.lidar_offset_x = 0.0 # m (NED convention) - not used currently
        self.lidar_offset_y = 0.0 # m (NED convention) - not used currently
        self.angles = None # To be populated on first scan
        self.lidar_angle_offset = 0.0 * np.pi/180.0 # rad Adjust this value as needed for the sensor's rotation (maybe NED convention)

        # these parameters are used for clustering detected obstacles point cloud into straight lines using DBSCAN
        self.min_samples = 5    # Minimum number of points in a cluster
        self.eps = 0.2        # Maximum distance between points in a cluster (adjust as needed)
        self.max_angle_merge = 75.0 * np.pi/18.0  # Maximum angle difference to consider merging two line segments (radians), if the angle difference between 2 consecutive line segments is more than this, they will not be merged and consided seperate segments

        self.x_location = 0.0   # store lidar location data
        self.y_location = 0.0
        self.z_location = 0.0
        self.ranges = []        # store lidar range data
        self.angles = []        # store lidar angles
        self.w = 0.0            # store quaternion orientation data
        self.x = 0.0 
        self.y = 0.0
        self.z = 0.0

        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0

        self.actual_joint_angle = 0.0  # store actual joint angle data

        self.subscription  # prevent unused variable warning
        self.csv_file = None
        self.csv_writer = None
        self.first_scan = True
        self.filepath = home_path+'/carl_ws/src/carl_ws_src/morphocopter_planning/acados_mpc3/log/scan_data.csv'
        self.filepath2 = home_path+'/carl_ws/src/carl_ws_src/morphocopter_planning/acados_mpc3/log/line_segmenta.csv'

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=10)
        
        self.vehicle_local_position_subscription = self.create_subscription(VehicleLocalPosition,'/folding_drone/out/vehicle_local_position',self.vehicle_local_position_callback, qos_profile=qos_policy)  
        self.vehicle_orientation_subscription = self.create_subscription(FDOrientationNED, '/folding_drone/out/vehicle_orientation', self.vehicle_orientation_callback, qos_profile=qos_policy)

        # subscribe to the joint angle command to offset the front axis due to joint angle change
        self.actual_joint_angle_subscriptions = self.create_subscription(Float32, '/folding_drone/out/actual_joint_angle', self.actual_joint_angle_callback, qos_profile=qos_policy)


        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_orientation = FDOrientationNED()
        self.vehicle_local_position.x = 0.0
        self.vehicle_local_position.y = 0.0
        self.vehicle_local_position.z = 0.0
        self.vehicle_orientation.q = [1.0, 0.0, 0.0, 0.0]

    def open_csv(self):
        try:
            self.csv_file = open(self.filepath, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.get_logger().info(f'Opened CSV file: {self.filepath}')

            self.csv_file2 = open(self.filepath2, 'w', newline='')
            self.csv_writer2 = csv.writer(self.csv_file2)
            self.get_logger().info(f'Opened CSV file: {self.filepath2}')

        except Exception as e:
            self.get_logger().error(f'Failed to open CSV file: {e}')

    def write_header(self, msg):
         header = ['timestamp', 'angle_min', 'angle_max', 'angle_increment', 'time_increment', 'scan_time', 'range_min', 'range_max', 'joint_angle', 'location x (NED)', 'location y (NED)', 'location z (NED)', 'orientation w (NED)', 'orientation x (NED)', 'orientation y (NED)', 'orientation z (NED)', 'Joint Angle (rad)'] + [ f'range_{i}' for i in range(len(msg.ranges))]
         header2 = ['x_location', 'y_location', 'z_location', 'start_x', 'start_y', 'start_z', 'start_cluster_id', 'end_x', 'end_y', 'end_z', 'end_cluster_id', 'and so on ...']
         try:
             self.csv_writer.writerow(header)
             self.get_logger().info('Wrote header to CSV')

             self.csv_writer2.writerow(header2)
             self.get_logger().info('Wrote header to CSV for line segments')
             
         except Exception as e:
             self.get_logger().error(f'Failed to write header: {e}')

    def write_scan_data(self,msg):
        row = [msg.header.stamp.sec + msg.header.stamp.nanosec/1e9,
               msg.angle_min,
               msg.angle_max,
               msg.angle_increment,
               msg.time_increment,
               msg.scan_time,
               msg.range_min,
               msg.range_max,
               self.actual_joint_angle,
               self.vehicle_local_position.x,
               self.vehicle_local_position.y,
               self.vehicle_local_position.z,
               self.vehicle_orientation.q[0],
               self.vehicle_orientation.q[1],
               self.vehicle_orientation.q[2],
               self.vehicle_orientation.q[3]] + list(msg.ranges)
        try:
            self.csv_writer.writerow(row)
        except Exception as e:
            self.get_logger().error(f'Failed to write data: {e}')

    def write_line_segment_data(self):
        row_to_write = [self.x_location, self.y_location, self.z_location]
        num_segments = self.line_segment.shape[2]
        for i in range(num_segments):
            start_point = self.line_segment[0, :, i]
            end_point = self.line_segment[1, :, i]
            row_to_write.append(start_point[0])
            row_to_write.append(start_point[1])
            row_to_write.append(start_point[2])
            row_to_write.append(start_point[3])
            row_to_write.append(end_point[0])
            row_to_write.append(end_point[1])
            row_to_write.append(end_point[2])
            row_to_write.append(end_point[3])
    
        try:
            self.csv_writer2.writerow(row_to_write)
        except Exception as e:
            self.get_logger().error(f'Failed to write line segment data: {e}')



    def listener_callback_simulation(self, msg):
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment

        self.x_location = self.vehicle_local_position.y
        self.y_location = self.vehicle_local_position.x
        self.z_location = -self.vehicle_local_position.z

        # The segmentation etc is done in ENU coordinates and transformed to NED before publishing line_segments

        quat_lidar_offset = [0.70710678, 
                             0.0, 
                             0.0, 
                             0.70710678]
        
        quat_vehicle_orientation = [self.vehicle_orientation.q[0], self.vehicle_orientation.q[2], self.vehicle_orientation.q[1], -self.vehicle_orientation.q[3]]

        quat_final = quaternion_multiply(quat_vehicle_orientation, quat_lidar_offset)

        self.w = quat_final[0]
        self.x = quat_final[1]
        self.y = quat_final[2]
        self.z = quat_final[3]
        self.ranges = msg.ranges
        msg_forward = msg
        self.process_lidar_data(msg_forward)

    def actual_joint_angle_callback(self,msg):
        self.actual_joint_angle = msg.data

    def listener_callback_hardware(self, msg):
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.x_location = self.vehicle_local_position.y
        self.y_location = self.vehicle_local_position.x
        self.z_location = -self.vehicle_local_position.z

        # The segmentation etc is done in ENU coordinates and transformed to NED before publishing line_segments

        quat_lidar_offset = [0.3007058, 
                             0.0, 
                             0.0, 
                             0.953717]  # Quaternion representing the fixed offset of the LiDAR from the vehicle frame (w, x, y, z)
        
        # quat_joint_angle_offset = [math.cos(self.actual_joint_angle / 2.0), 
        #                            0.0, 
        #                            0.0, 
        #                            math.sin(self.actual_joint_angle / 2.0)]  # Quaternion representing the joint angle offset (w, x, y, z)

        quat_vehicle_orientation = [self.vehicle_orientation.q[0], self.vehicle_orientation.q[2], self.vehicle_orientation.q[1], -self.vehicle_orientation.q[3]]

        # quat_final = quaternion_multiply(quat_vehicle_orientation, 
        #                                  quaternion_multiply(quat_joint_angle_offset, quat_lidar_offset))
        
        quat_final = quaternion_multiply(quat_vehicle_orientation, quat_lidar_offset)

        self.w = quat_final[0]
        self.x = quat_final[1]
        self.y = quat_final[2]
        self.z = quat_final[3]

        self.ranges = msg.ranges
        msg_forward = msg
        self.process_lidar_data(msg_forward)
        
    
    def process_lidar_data(self,msg_forward):
        # Convert quaternion to rotation matrix

        if self.first_scan:
            self.open_csv()
            self.write_header(msg_forward)
            self.first_scan = False
        self.write_scan_data(msg_forward)

        if float(self.w) == 0.0:
            return  # Skip if quaternion is zero to avoid division by zero
    
        rotation_matrix = q2rotmat([float(self.w), float(self.x), float(self.y), float(self.z)]) 
        
        # rotation_matrix = np.matmul(rotation_matrix, np.array([[np.cos(self.lidar_angle_offset), np.sin(self.lidar_angle_offset), 0], 
        #                                                    [-np.sin(self.lidar_angle_offset), np.cos(self.lidar_angle_offset), 0], 
        #                                                    [0, 0, 1]])) 
        

        # Verifying with visualize_scan.py, this joint angle based adjustment is not needed somehow
        # joint_angle_axis_offset = np.pi/4.0 - self.actual_joint_angle/2.0  # Adjust the rotation matrix based on the actual joint angle
        # rotation_matrix = np.matmul(rotation_matrix, np.array([[np.cos(joint_angle_axis_offset), np.sin(joint_angle_axis_offset), 0], 
        #                                                    [-np.sin(joint_angle_axis_offset), np.cos(joint_angle_axis_offset), 0], 
        #                                                    [0, 0, 1]]))  # due to joint angle change
        
        # Filter out infinite range values
        self.ranges = np.array(self.ranges)
        angles = np.linspace(self.angle_min, self.angle_max, len(self.ranges))


        # Changes made to consider only points within certain range based on active waypoints horizon and joint angle obstacle consideration distance
        self.valid_indices = np.where((self.ranges != float('inf')) & (~np.isnan(self.ranges)))[0]
        self.valid_ranges = self.ranges[self.valid_indices]
        valid_angles = angles[self.valid_indices]

        lidar_range_consideration = 1.5 * max(active_waypoints_horizon_min/linear_scale, joint_angle_obstacle_consideration_dist/linear_scale, 2.5)
        ############### DO NOT USE FOR HARDWARE: ######################
        # lidar_range_consideration = np.inf 
        ###############################################################
        self.considered_indices = np.where(self.valid_ranges <= lidar_range_consideration)[0]

        self.ranges = self.valid_ranges[self.considered_indices]
        angles = valid_angles[self.considered_indices]
        


        # Convert ranges and angles to cartesian coordinates
        
        

        xs = self.ranges * np.cos(angles)   # minus because angles are in ENU frame, but we are working in NED frame
        ys = self.ranges * np.sin(angles)
        zs = np.ones(len(self.ranges)) * self.lidar_offset_z

        # Apply the rotation
        points = np.stack([xs, ys, zs], axis=1)
    
        rotated_points = np.dot(points, rotation_matrix.T)
        
        self.x_rotated = rotated_points[:, 0] + self.x_location
        self.y_rotated = rotated_points[:, 1] + self.y_location
        self.z_rotated = rotated_points[:, 2] + self.z_location

        

        self.cluster_points()

    
    def cluster_points(self):
        # Prepare the data for clustering
        data = np.column_stack([self.x_rotated, self.y_rotated, self.z_rotated, self.ranges, self.considered_indices])  # Include ranges as a feature

        # Perform DBSCAN clustering to identify individual lines
        if data.shape[0] == 0:
            clusters = np.array([])
        else:
            dbscan = DBSCAN(eps = self.eps, min_samples = self.min_samples)  # Adjust eps and min_samples as needed
            clusters = dbscan.fit_predict(data[:, :3])  # Use only the first three columns (x, y, z) for clustering

        self.line_segment = np.zeros((2, 4, 0))  # Initialize line segment variable

        # Iterate through each cluster and fit a line segment
        for cluster_id in np.unique(clusters):
            if cluster_id == -1:  # Skip noise points
                continue

            # Extract the points belonging to the current cluster
            cluster_points = data[clusters == cluster_id]

            # Sort cluster points by their original index before splitting
            cluster_points = cluster_points[cluster_points[:, 4].argsort()]

            # Break down cluster_points based on distance trend changes
            break_indices = []
            prev_trend = None
            window = 3  # Number of points to consider for moving average/trend

            distances = cluster_points[:, 3]
            for i in range(window, len(distances) - window):
                prev_avg = np.mean(distances[i - window:i])
                next_avg = np.mean(distances[i:i + window])
                trend = np.sign(next_avg - prev_avg)
                if prev_trend is not None and trend != 0 and trend != prev_trend:
                    break_indices.append(i)
                if trend != 0:
                    prev_trend = trend
            
            # Add breaks based on large jumps in 3D distance
            for i in range(1, len(cluster_points)):
                p1 = cluster_points[i-1, :3]
                p2 = cluster_points[i, :3]
                dist = np.linalg.norm(p1 - p2)
                if dist > self.eps * 2: # Use a slightly larger threshold than clustering eps
                    if i not in break_indices:
                        break_indices.append(i)
            
            break_indices.sort()  # Keep indices in order

            # Split cluster_points at break_indices
            split_points = np.split(cluster_points, break_indices) if break_indices else [cluster_points]
            for sub_cluster in split_points:
                if len(sub_cluster) < 2:
                    continue  # Skip too small clusters

                # Perform linear regression to fit a line
                linear_regressor = LinearRegression()
                linear_regressor.fit(sub_cluster[:, :2], sub_cluster[:, 2])  # Fit X and Y to Z

                # Define the endpoints of the line segment using the first and last points
                # of the ordered sub-cluster.
                x_start, y_start = sub_cluster[0, 0], sub_cluster[0, 1]
                x_end, y_end = sub_cluster[-1, 0], sub_cluster[-1, 1]

                # Predict the z values for the endpoints
                z_start = linear_regressor.predict([[x_start, y_start]])[0]
                z_end = linear_regressor.predict([[x_end, y_end]])[0]

                self.line_segment = np.dstack((self.line_segment, np.array([[x_start, y_start, z_start, cluster_id], [x_end, y_end, z_end, cluster_id]])))

            # # Merge segments logic
            # if self.line_segment.shape[2] > 1:
            #     merged_segments = []
            #     used_indices = set()
            #     num_segments = self.line_segment.shape[2]

            #     for i in range(num_segments):
            #         if i in used_indices:
            #             continue

            #         current_segment = self.line_segment[:, :, i]
                    
            #         for j in range(i + 1, num_segments):
            #             if j in used_indices:
            #                 continue

            #             other_segment = self.line_segment[:, :, j]
            #             min_dist = np.inf

            #             # Check for collinearity and proximity
            #             v1 = current_segment[1, :3] - current_segment[0, :3]
            #             v2 = other_segment[1, :3] - other_segment[0, :3]
                        
            #             norm_v1 = np.linalg.norm(v1)
            #             norm_v2 = np.linalg.norm(v2)

            #             if norm_v1 == 0 or norm_v2 == 0:
            #                 continue

            #             # Angle between vectors
            #             cos_angle = np.dot(v1, v2) / (norm_v1 * norm_v2)
            #             angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))

            #             # Check if they are nearly parallel (angle close to 0 or pi)
            #             if angle < 0.1 or angle > np.pi - 0.1:
            #             # Check distance between endpoints
            #                 endpoints1 = [current_segment[0, :3], current_segment[1, :3]]
            #                 endpoints2 = [other_segment[0, :3], other_segment[1, :3]]
            #                 min_dist = np.inf
            #                 for p1 in endpoints1:
            #                     for p2 in endpoints2:
            #                         dist = np.linalg.norm(p1 - p2)
            #                         if dist < min_dist:
            #                             min_dist = dist
                        
            #             if min_dist < self.eps:
            #                 # Merge segments
            #                 all_points = np.vstack([endpoints1, endpoints2])
            #                 distances = np.linalg.norm(all_points[:, np.newaxis, :] - all_points[np.newaxis, :, :], axis=2)
            #                 max_dist_idx = np.unravel_index(np.argmax(distances), distances.shape)
                            
            #                 new_start = all_points[max_dist_idx[0]]
            #                 new_end = all_points[max_dist_idx[1]]
                            
            #                 # Update current segment and mark the other as used
            #                 current_segment = np.array([[new_start[0], new_start[1], new_start[2], current_segment[0, 3]],
            #                             [new_end[0], new_end[1], new_end[2], current_segment[1, 3]]])
            #                 used_indices.add(j)

            #         merged_segments.append(current_segment)
            #         used_indices.add(i)
                    
            #     if merged_segments:
            #         self.line_segment = np.transpose(np.array(merged_segments), (1, 2, 0))
    
        self.publish_segments()
    
    def publish_segments(self):
        # Reshape the line segment data for publishing
        # self.line_segment has shape (2, 3, num_segments)
        # We transpose it to (num_segments, 2, 3) for a more intuitive layout
        if self.line_segment.shape[2] == 0:
            return # Nothing to publish
        
        # Filter out segments where both endpoints are below ground level (z < 0.01 in ENU frame)
        mask = ~((self.line_segment[0, 2, :] < 0.01) & (self.line_segment[1, 2, :] < 0.01))
        self.line_segment = self.line_segment[:, :, mask]
        
        self.write_line_segment_data()  # Write line segment data to CSV
        
        
        
        # Publish the location where the segments were captured
        current_location_ned = [self.y_location, self.x_location, -self.z_location]
        location_msg = Float64MultiArray()
        location_msg.data = current_location_ned
        self.segment_capture_location_publisher.publish(location_msg)

        # Convert ENU to NED coordinates
        # ENU (x, y, z) -> NED (y, x, -z)
        # The shape of self.line_segment is (2, 4, num_segments)
        # where the 4 columns are [x, y, z, cluster_id]
        line_segment_ned = np.copy(self.line_segment)
        line_segment_ned[:, 0, :] = self.line_segment[:, 1, :]  # NED x = ENU y
        line_segment_ned[:, 1, :] = self.line_segment[:, 0, :]  # NED y = ENU x
        line_segment_ned[:, 2, :] = -self.line_segment[:, 2, :] # NED z = -ENU z
        # The 4th column (cluster_id) remains unchanged.
        line_segment_ned[:, 3, :] = self.line_segment[:, 3, :]

        transposed_segments = np.transpose(line_segment_ned, (2, 0, 1))
        num_segments, num_points, num_coords = transposed_segments.shape

        # Create the Float64MultiArray message
        msg = Float64MultiArray()

        # Set up the layout
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "segments"
        msg.layout.dim[0].size = num_segments
        msg.layout.dim[0].stride = num_segments * num_points * num_coords

        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[1].label = "points"
        msg.layout.dim[1].size = num_points
        msg.layout.dim[1].stride = num_points * num_coords

        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[2].label = "coordinates_and_cluster_id"
        msg.layout.dim[2].size = num_coords
        msg.layout.dim[2].stride = num_coords
        
        msg.layout.data_offset = 0

        # Flatten the array and fill the data field
        msg.data = transposed_segments.flatten().tolist()

        # Publish the message
        self.segment_publisher.publish(msg)
    
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_orientation_callback(self, msg):
        self.vehicle_orientation = msg

def main(args=None):
    rclpy.init(args=args)
    scan_subscriber = ScanSubscriber()
    rclpy.spin(scan_subscriber)
    scan_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()