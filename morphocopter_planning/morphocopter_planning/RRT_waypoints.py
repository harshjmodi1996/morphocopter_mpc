import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
import numpy as np
import random
import pybullet as p
import pybullet_data
import xml.etree.ElementTree as ET
from tf_transformations import quaternion_from_euler

class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_waypoint_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'waypoints', 10)
        self.start = np.array([0.0, 0.0, 1.0, 0.78])  # x, y, z, yaw
        self.goal = np.array([0.165, 6.85, 1.0 , 0.78])  # x, y, z, yaw
        self.bounds = [[-1.7, 2.038], [-2.0, 9.24], [0.9, 1.1], [0.77,0.79]]  # x, y, z, yaw
        self.obstacles = self.load_obstacles()
        self.max_iter = 4000
        self.step_size = 0.1
        self.collision_check_distance = 0.01
        self.robot_id = None
        self.planning_attempts = 0
        self.max_planning_attempts = 20  # Limit planning attempts

        # Initialize PyBullet
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Load the robot *without* fixed base.  This is CRUCIAL for attitude control.
        self.robot_id = p.loadURDF("/home/harshmodi/PX4-Autopilot/Tools/simulation/gz/models/folding_drone_base_v2/model.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=False)

        # Load obstacles from URDF
        self.obstacle_ids = []
        self.obstacle_ids.append(p.loadURDF("/home/harshmodi/PX4-Autopilot/Tools/simulation/gz/models/folding_drone_obstacle1/model.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0])))

        self.plan()

    def load_obstacles(self):
        # This function is not used anymore, obstacles are loaded directly in __init__
        return []

    def is_collision(self, point, yaw):  # Added yaw
        x, y, z = point
        orientation = quaternion_from_euler(0, 0, yaw)
        p.resetBasePositionAndOrientation(self.robot_id, [x, y, z], orientation)
        p.performCollisionDetection()
        for obstacle_id in self.obstacle_ids:
            closest_points = p.getClosestPoints(self.robot_id, obstacle_id, self.collision_check_distance)
            if len(closest_points) > 0:
                return True
        return False

    def get_random_point(self):
        return np.array([
            random.uniform(self.bounds[0][0], self.bounds[0][1]),
            random.uniform(self.bounds[1][0], self.bounds[1][1]),
            random.uniform(self.bounds[2][0], self.bounds[2][1]),
            random.uniform(self.bounds[3][0], self.bounds[3][1])  # Yaw
        ])

    def get_random_orientation(self):
        # Generate a random orientation (Euler angles)
        roll = random.uniform(-np.pi, np.pi)
        pitch = random.uniform(-np.pi, np.pi)
        yaw = random.uniform(-np.pi, np.pi)
        return quaternion_from_euler(roll, pitch, yaw)

    def nearest_neighbor(self, nodes, point):
        # Compare x, y, z only
        return min(nodes, key=lambda n: np.linalg.norm(n[:3] - point[:3]))

    def steer(self, from_node, to_node):
        direction = to_node[:3] - from_node[:3]  # Steer towards x, y, z only
        distance = np.linalg.norm(direction)
        if distance > self.step_size:
            direction = direction / distance * self.step_size
        new_position = from_node[:3] + direction

        # Steer the yaw angle
        yaw_direction = to_node[3] - from_node[3]
        if abs(yaw_direction) > np.pi:
            yaw_direction -= np.sign(yaw_direction) * 2 * np.pi  # Handle angle wrapping
        new_yaw = from_node[3] + np.clip(yaw_direction, -self.step_size, self.step_size)

        new_node = np.array([new_position[0], new_position[1], new_position[2], new_yaw])
        return new_node

    def is_path_valid(self, from_node, to_node, num_samples=10):
        for i in range(num_samples + 1):
            alpha = i / num_samples
            interpolated_point = (1 - alpha) * from_node[:3] + alpha * to_node[:3]
            interpolated_yaw = (1 - alpha) * from_node[3] + alpha * to_node[3]
            if self.is_collision(interpolated_point, interpolated_yaw):
                return False
        return True

    def plan(self):
        self.planning_attempts = 0
        while self.planning_attempts < self.max_planning_attempts:
            self.planning_attempts += 1
            self.get_logger().info(f'Planning attempt: {self.planning_attempts}')
            start_orientation = 0.0  # Initial yaw
            nodes = [self.start]
            parents = {tuple(self.start): None}
            success = False

            for _ in range(self.max_iter):
                print(_)
                random_point = self.get_random_point()
                # print(random_point)
                nearest_node = self.nearest_neighbor(nodes, random_point)
                new_node = self.steer(nearest_node, random_point)

                if not self.is_collision(new_node[:3], new_node[3]) and self.is_path_valid(nearest_node, new_node):
                    nodes.append(new_node)
                    parents[tuple(new_node)] = nearest_node

                    if np.linalg.norm(new_node[:3] - self.goal[:3]) < self.step_size:
                        self.get_logger().info('Goal Reached!')
                        success = True
                        # Add the goal to parents, with the new_node as its parent
                        parents[tuple(self.goal)] = new_node
                        break  # Break out of the inner loop

            if success:
                path = self.reconstruct_path(parents)
                self.publish_path(path)
                break  # Break out of the outer loop
            else:
                self.get_logger().warn('RRT Failed to find a path. Retrying...')

        if self.planning_attempts >= self.max_planning_attempts:
            self.get_logger().error('RRT Failed to find a path after multiple attempts.')

    def reconstruct_path(self, parents):
        path = [self.goal]
        current = self.goal
        while tuple(current) != tuple(self.start):
            current = parents[tuple(current)]
            path.append(current)
        path.reverse()
        return path

    def publish_path(self, path):
        for point in path:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"  # Replace with your frame ID
            pose_stamped.pose.position.x = float(point[0])
            pose_stamped.pose.position.y = float(point[1])
            pose_stamped.pose.position.z = float(point[2])

            # Convert yaw to quaternion
            q = quaternion_from_euler(0, 0, point[3])
            pose_stamped.pose.orientation.x = float(q[0])
            pose_stamped.pose.orientation.y = float(q[1])
            pose_stamped.pose.orientation.z = float(q[2])
            pose_stamped.pose.orientation.w = float(q[3])

            self.publisher_.publish(pose_stamped)
            self.get_logger().info(f'Publishing waypoint: {point}')

def main(args=None):
    rclpy.init(args=args)
    rrt_planner = RRTPlanner()
    rclpy.spin(rrt_planner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()