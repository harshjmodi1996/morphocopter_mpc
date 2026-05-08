import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Vector3
from folding_drone_msgs.msg import FD4DPose, FD4DPoseArray

import ompl.base as ob
import ompl.geometric as og
import pybullet as p
import pybullet_data
import numpy as np
from tf_transformations import quaternion_from_euler

class OMPLPlanner(Node):
    def __init__(self):
        super().__init__('OMPL_planner')
        self.publisher_ = self.create_publisher(FD4DPoseArray, '/folding_drone/path_planner_waypoints', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.waypoint_index = 0
        self.path_poses = []
        self.solution_found = False  # Add a flag to indicate if a solution has been found

        # Initialize PyBullet
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setRealTimeSimulation(0)
        self.robot_id = p.loadURDF("/home/harshmodi/PX4-Autopilot/Tools/simulation/gz/models/folding_drone_base_v2/model.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

        # Get joint index for upper_arm_joint
        self.upper_arm_joint_index = self.get_joint_index("upper_arm_joint")
        if self.upper_arm_joint_index is None:
            self.get_logger().error("Failed to find upper_arm_joint in URDF.")

        # Load obstacles f00000000000000000000000000000000000000rom URDF
        self.obstacle_ids = []
        self.obstacle_ids.append(p.loadURDF("/home/harshmodi/PX4-Autopilot/Tools/simulation/gz/models/cluttered_obstacles_3dgs/model.urdf", [0, 0, 0.5], p.getQuaternionFromEuler([0, 0, 0])))

        # # Define the space bounds - for others
        # self.space_bounds = ob.RealVectorBounds(5)
        # self.space_bounds.setLow(0, 0.0)   # x
        # self.space_bounds.setLow(1, -2.4)   # y
        # self.space_bounds.setLow(2, 1.5)      # z
        # self.space_bounds.setLow(3, 1.57)    # yaw
        # self.space_bounds.setLow(4, 1.57)   # arm
        # self.space_bounds.setHigh(0, 10.0) # x
        # self.space_bounds.setHigh(1, 2.4)  # y
        # self.space_bounds.setHigh(2, 1.5)   # z
        # self.space_bounds.setHigh(3, 1.57)   # yaw
        # self.space_bounds.setHigh(4, 1.57)  # arm

        # # Define the space bounds - for obstacle 2 for first half of trajectory (same as TMECH paper)
        # self.space_bounds = ob.RealVectorBounds(5)
        # self.space_bounds.setLow(0, 0.0)   # x
        # self.space_bounds.setLow(1, -2.4)   # y
        # self.space_bounds.setLow(2, 0.0)      # z
        # self.space_bounds.setLow(3, 1.57)    # yaw
        # self.space_bounds.setLow(4, 1.57)   # arm
        # self.space_bounds.setHigh(0, 11.0) # x
        # self.space_bounds.setHigh(1, 2.4)  # y
        # self.space_bounds.setHigh(2, 3.0)   # z
        # self.space_bounds.setHigh(3, 1.57)   # yaw
        # self.space_bounds.setHigh(4, 1.57)  # arm

        # # Define the space bounds - for obstacle 2 for second half of trajectory (same as TMECH paper)
        # self.space_bounds = ob.RealVectorBounds(5)
        # self.space_bounds.setLow(0, 9.0)   # x
        # self.space_bounds.setLow(1, -2.4)   # y
        # self.space_bounds.setLow(2, 0.0)      # z
        # self.space_bounds.setLow(3, 0.0)    # yaw
        # self.space_bounds.setLow(4, 1.57)   # arm
        # self.space_bounds.setHigh(0, 11.0) # x
        # self.space_bounds.setHigh(1, 7.0)  # y
        # self.space_bounds.setHigh(2, 3.0)   # z
        # self.space_bounds.setHigh(3, 0.0)   # yaw
        # self.space_bounds.setHigh(4, 1.57)  # arm

        # Define the space bounds - for hardware cluttered env
        self.space_bounds = ob.RealVectorBounds(5)
        self.space_bounds.setLow(0, -0.8)   # x
        self.space_bounds.setLow(1, 0.0)   # y
        self.space_bounds.setLow(2, 1.2)      # z
        self.space_bounds.setLow(3, -0.5)    # yaw
        self.space_bounds.setLow(4, 1.57)   # arm
        self.space_bounds.setHigh(0, 0.8) # x
        self.space_bounds.setHigh(1, 4.25)  # y
        self.space_bounds.setHigh(2, 1.2)   # z
        self.space_bounds.setHigh(3, 0.5)   # yaw
        self.space_bounds.setHigh(4, 1.57)  # arm

        # # Define the space bounds - for hardware experiment related
        # self.space_bounds = ob.RealVectorBounds(5)
        # self.space_bounds.setLow(0, -0.4)   # x
        # self.space_bounds.setLow(1, -0.5)   # y
        # self.space_bounds.setLow(2, 0.3)      # z
        # self.space_bounds.setLow(3, 0.0)    # yaw
        # self.space_bounds.setLow(4, 1.57)   # arm
        # self.space_bounds.setHigh(0, 0.4) # x
        # self.space_bounds.setHigh(1, 3.5)  # y
        # self.space_bounds.setHigh(2, 2.0)   # z
        # self.space_bounds.setHigh(3, 0.0)   # yaw
        # self.space_bounds.setHigh(4, 1.57)  # arm

        # Define the state space
        self.space = ob.RealVectorStateSpace(5)
        self.space.setBounds(self.space_bounds)

        # Define the start and goal states
        self.start = ob.State(self.space)
        self.goal = ob.State(self.space)

        # # for others
        # self.start[0], self.start[1], self.start[2], self.start[3], self.start[4] = 0.0, 0.0, 1.5, 1.57, 1.57
        # self.goal[0], self.goal[1], self.goal[2], self.goal[3], self.goal[4] = 8.5, 0.0, 1.5, 1.57, 1.57

        # # for obstacle2 for first half of trajectory- same as TMECH paper 
        # self.start[0], self.start[1], self.start[2], self.start[3], self.start[4] = 0.0, 0.0, 0.3, 1.57, 1.57
        # self.goal[0], self.goal[1], self.goal[2], self.goal[3], self.goal[4] = 9.5, -1.5, 0.3, 1.57, 1.57

        # # for obstacle2 for second half of trajectory- same as TMECH paper 
        # self.start[0], self.start[1], self.start[2], self.start[3], self.start[4] = 9.5, -1.5, 1.29, 0.0, 1.57
        # self.goal[0], self.goal[1], self.goal[2], self.goal[3], self.goal[4] = 10.0, 6.0, 0.3, 0.0, 1.57

        # for cluttered hardware exp
        self.start[0], self.start[1], self.start[2], self.start[3], self.start[4] = 0.0, 0.0, 1.2, 0.0, 1.57
        self.goal[0], self.goal[1], self.goal[2], self.goal[3], self.goal[4] = 0.0, 4.25, 1.2, 0.0, 1.57

        # # for hardware experiment related
        # self.start[0], self.start[1], self.start[2], self.start[3], self.start[4] = 0.0, 0.0, 0.3, 0.0, 1.57
        # self.goal[0], self.goal[1], self.goal[2], self.goal[3], self.goal[4] = 0.0, 3.0, 0.3, 0.0, 1.57

        # Define the problem
        self.si = ob.SpaceInformation(self.space)
        self.collision_check_distance = 0.01
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))
        self.problem = ob.ProblemDefinition(self.si)
        self.problem.setStartAndGoalStates(self.start, self.goal)

        # Define the planner
        self.planner = og.BITstar(self.si)
        self.planner.setProblemDefinition(self.problem)
        self.planner.setup()

    def get_joint_index(self, joint_name):
        """Helper function to get the joint index by name."""
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            if joint_info[1].decode("utf-8") == joint_name:
                return joint_info[0]
        return None

    def is_state_valid(self, state):
        pos = [state[0], state[1], state[2]]
        yaw = state[3]
        arm_angle = state[4]
        quaternion = quaternion_from_euler(0, 0, yaw - arm_angle / 2.0 + np.pi / 2.0)
        p.resetBasePositionAndOrientation(self.robot_id, pos, quaternion)
        p.setJointMotorControl2(self.robot_id, self.upper_arm_joint_index, p.POSITION_CONTROL, targetPosition=arm_angle)
        p.stepSimulation()
        p.performCollisionDetection()
        for obstacle_id in self.obstacle_ids:
            closest_points = p.getClosestPoints(bodyA=self.robot_id, bodyB=obstacle_id, distance=self.collision_check_distance)
            if len(closest_points) > 0:
                return False
        return True

    def timer_callback(self):
        if self.solution_found:  # If a solution has already been found, stop the timer
            self.destroy_timer(self.timer)
            self.get_logger().info('Solution found and published. Stopping the planner.')
            return

        if not self.path_poses:
            solved = self.planner.solve(10.0)
            if solved:
                path = self.problem.getSolutionPath()
                self.path_poses = []
                self.yaws = []

                for i in range(path.getStateCount()):
                    state = path.getState(i)
                    real_state = self.space.allocState()
                    self.space.copyState(real_state, state)
                    pos = [real_state[0], real_state[1], real_state[2]]
                    yaw = real_state[3]

                    self.path_poses.append(pos)
                    self.yaws.append(yaw)

                    self.space.freeState(real_state)

                publication_msg = FD4DPoseArray()
                publication_msg.poses = []

                for i in range(1, len(self.path_poses)):
                    p0 = self.path_poses[i - 1]
                    p1 = self.path_poses[i]
                    direction_vector = np.array([p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]])
                    line_length = np.linalg.norm(direction_vector)
                    direction_vector = direction_vector / line_length
                    num_points = int(line_length / 0.05)  # Number of points to sample along the line
                    
                    for j in range(1, num_points + 1):
                        point_on_line = np.array([p0[0], p0[1], p0[2]]) + direction_vector * j * 0.05
                        interpolated_yaw = self.yaws[i - 1] + (self.yaws[i] - self.yaws[i - 1]) * (j / num_points)
                        interpolated_yaw = interpolated_yaw % (2 * np.pi)  # Normalize yaw to [0, 2π]

                        interpolated_pose = FD4DPose()
                        interpolated_pose.position = Point(x=point_on_line[0], y=point_on_line[1], z=point_on_line[2])
                        interpolated_pose.yaw = interpolated_yaw
                        publication_msg.poses.append(interpolated_pose)
                self.publisher_.publish(publication_msg)

                self.waypoint_index = 0
                self.get_logger().info('Found and published a valid path.')
                self.solution_found = True  # Set the flag to True after finding a solution
                self.destroy_timer(self.timer)  # Stop the timer
                self.get_logger().info('Solution found and published. Stopping the planner.')
            else:
                self.get_logger().warn('Failed to find a solution.')
            return

        if self.waypoint_index < len(self.path_poses):
            pos = [self.path_poses[self.waypoint_index].position.x,
                   self.path_poses[self.waypoint_index].position.y,
                   self.path_poses[self.waypoint_index].position.z]
            quaternion = [self.path_poses[self.waypoint_index].orientation.x,
                          self.path_poses[self.waypoint_index].orientation.y,
                          self.path_poses[self.waypoint_index].orientation.z,
                          self.path_poses[self.waypoint_index].orientation.w]
            arm_angle = self.path_poses[self.waypoint_index].joint_angle
            p.resetBasePositionAndOrientation(self.robot_id, pos, quaternion)
            p.setJointMotorControl2(self.robot_id, self.upper_arm_joint_index, p.POSITION_CONTROL, targetPosition=arm_angle)
            p.stepSimulation()
            self.get_logger().info(f'Executing waypoint {self.waypoint_index}: pos={pos}, arm={arm_angle}')
            self.waypoint_index += 1
        else:
            self.get_logger().info('Path execution complete.')
            self.path_poses = []
            self.waypoint_index = 0

def main(args=None):
    rclpy.init(args=args)
    OMPL_planner = OMPLPlanner()
    rclpy.spin(OMPL_planner)
    OMPL_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()