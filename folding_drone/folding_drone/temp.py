import rclpy
from rclpy.node import Node
import pybullet as p
import pybullet_data
import time
from std_msgs.msg import Float32

class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control_node')

        # PyBullet setup
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, 0.0)
        p.setRealTimeSimulation(0)  # Disable real-time simulation

        # Load the robot
        self.robot_id = p.loadURDF("/home/harshmodi/PX4-Autopilot/Tools/simulation/gz/models/folding_drone_base_v2/model.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

        # Get joint index
        self.upper_arm_joint_index = self.get_joint_index("upper_arm_joint")
        if self.upper_arm_joint_index is None:
            self.get_logger().error("Failed to find upper_arm_joint in URDF.")
            rclpy.shutdown()
            return

        # Default desired arm angle (in radians)
        self.desired_arm_angle = 0.0

        # Create a subscriber to /folding_drone/temp/joint_angle
        self.subscription = self.create_subscription(
            Float32,
            '/folding_drone/temp/joint_angle',
            self.joint_angle_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create a timer to control the arm
        self.timer = self.create_timer(0.02, self.control_arm)

    def get_joint_index(self, joint_name):
        """Helper function to get the joint index by name."""
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            if joint_info[1].decode("utf-8") == joint_name:
                return joint_info[0]
        return None

    def joint_angle_callback(self, msg):
        """Callback function to update the desired arm angle."""
        self.desired_arm_angle = msg.data
        self.get_logger().info(f"Received desired joint angle: {self.desired_arm_angle}")

    def control_arm(self):
        """Controls the arm to the desired angle."""
        p.setJointMotorControl2(self.robot_id,
                                self.upper_arm_joint_index,
                                p.POSITION_CONTROL,
                                targetPosition=self.desired_arm_angle)
        p.stepSimulation()
        time.sleep(0.02)  # Simulate a small delay

def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()