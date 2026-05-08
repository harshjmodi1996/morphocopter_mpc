import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
import numpy as np
import random
import pybullet as p
import pybullet_data
import xml.etree.ElementTree as ET
from tf_transformations import quaternion_from_euler

class RRTStarPlanner(Node):
    def __init__(self):
        super().__init__('rrt_star_waypoint_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'waypoints', 10)
        self.start = np.array([0.0, 0.0, 1.0, 0.78])  # x, y, z, yaw
        self.goal = np.array([0.165, 6.85, 1.0, 0.78])  # x, y, z, yaw
        self.bounds = [[-1.7, 2.038], [-2.0, 9.24], [0.9, 1.1], [0.77,0.79]]  # x, y, z, yaw
        self.obstacles = self.load_obstacles()
        self.max_iter = 4000
        self.step_size = 0.1
        self.collision_check_distance = 0.05
        self.robot_id = None
        self.planning_attempts = 0
        self.max_planning_attempts = 20  # Limit planning attempts
        self.rewire_radius = 0.5  # Radius for rewiring

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

    def cost(self, node, parents):
        """Calculates the cost from the start node to the given node."""
        cost = 0
        current = node
        while tuple(current) != tuple(self.start):
            if tuple(current) not in parents:
                return float('inf')  # Indicate an invalid path
            parent = parents[tuple(current)]
            cost += np.linalg.norm(current[:3] - parent[:3])  # Euclidean distance
            current = parent
        return cost

    def near_nodes(self, nodes, new_node):
        """Finds the nodes within a certain radius of the new node."""
        near_nodes = []
        for node in nodes:
            if np.linalg.norm(new_node[:3] - node[:3]) < self.rewire_radius:
                near_nodes.append(node)
        return near_nodes

    def plan(self):
        self.planning_attempts = 0
        while self.planning_attempts < self.max_planning_attempts:
            self.planning_attempts += 1
            self.get_logger().info(f'Planning attempt: {self.planning_attempts}')
            nodes = [self.start]
            parents = {tuple(self.start): None}
            costs = {tuple(self.start): 0.0}  # Cost from start to each node
            success = False
            goal_node = None  # Store the node closest to the goal

            for _ in range(self.max_iter):
                random_point = self.get_random_point()
                nearest_node = self.nearest_neighbor(nodes, random_point)
                new_node = self.steer(nearest_node, random_point)

                if not self.is_collision(new_node[:3], new_node[3]) and self.is_path_valid(nearest_node, new_node):
                    near = self.near_nodes(nodes, new_node)

                    # Choose parent from near nodes that minimizes cost
                    min_cost = self.cost(nearest_node, parents) + np.linalg.norm(new_node[:3] - nearest_node[:3])
                    best_parent = nearest_node

                    for near_node in near:
                        new_cost = self.cost(near_node, parents) + np.linalg.norm(new_node[:3] - near_node[:3])
                        if new_cost < min_cost and self.is_path_valid(near_node, new_node):
                            min_cost = new_cost
                            best_parent = near_node

                    # Add new node to the tree
                    nodes.append(new_node)
                    parents[tuple(new_node)] = best_parent
                    costs[tuple(new_node)] = min_cost

                    # Rewire the tree
                    for near_node in near:
                        existing_cost = self.cost(near_node, parents)
                        new_cost = self.cost(new_node, parents) + np.linalg.norm(new_node[:3] - near_node[:3])
                        if new_cost < existing_cost and self.is_path_valid(new_node, near_node):
                            parents[tuple(near_node)] = new_node
                            costs[tuple(near_node)] = new_cost

                    # Check if the new node is close to the goal
                    if np.linalg.norm(new_node[:3] - self.goal[:3]) < self.step_size:
                        self.get_logger().info('Near Goal!')
                        success = True
                        goal_node = new_node  # Store the node closest to the goal
                        parents[tuple(self.goal)] = new_node

            # After max_iter, check if a goal node was found
            if success:
                self.get_logger().info('Goal Reached!')
                path = self.reconstruct_path(parents)
                self.publish_path(path)
                break
            else:
                self.get_logger().warn('RRT* Failed to find a path. Retrying...')

        if self.planning_attempts >= self.max_planning_attempts:
            self.get_logger().error('RRT* Failed to find a path after multiple attempts.')

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
    rrt_star_planner = RRTStarPlanner()
    rclpy.spin(rrt_star_planner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()