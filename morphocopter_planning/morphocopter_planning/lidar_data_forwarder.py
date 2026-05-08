#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from folding_drone_msgs.msg import FDOrientationNED
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import Float32
import math


class LidarDataForwarder(Node):
    """
    A ROS2 node that forwards LaserScan messages from
    /model/folding_drone/lidar → /scan
    """

    def __init__(self):
        super().__init__('lidar_data_forwarder')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_broadcaster1 = TransformBroadcaster(self)

        # Create subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            '/model/folding_drone/lidar',
            self.lidar_callback_simulation,
            10
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback_hardware,
            10
        )

        # Create publisher
        self.publisher_simulation = self.create_publisher(
            LaserScan,
            '/scan_final_simulation',
            10
        )

        self.publisher_hardware = self.create_publisher(
            LaserScan,
            '/scan_final_hardware',
            10
        )
        
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=10)
        self.vehicle_orientation_subscription = self.create_subscription(FDOrientationNED, '/folding_drone/out/vehicle_orientation', self.vehicle_orientation_callback, qos_profile=qos_policy)
        self.vehicle_local_position_subscription = self.create_subscription(VehicleLocalPosition,'/folding_drone/out/vehicle_local_position',self.vehicle_local_position_callback, qos_profile=qos_policy)  

        # subscribe to the joint angle command to offset the front axis due to joint angle change
        self.actual_joint_angle_subscriptions = self.create_subscription(Float32, '/folding_drone/out/actual_joint_angle', self.actual_joint_angle_callback, qos_profile=qos_policy)

        
        self.vehicle_orientation = FDOrientationNED()
        self.vehicle_orientation.q = [1.0, 0.0, 0.0, 0.0]

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_local_position.x = 0.0
        self.vehicle_local_position.y = 0.0
        self.vehicle_local_position.z = 0.0

        self.actual_joint_angle = 0.0

        self.get_logger().info('Forwarding /model/folding_drone/lidar → /scan')

    def lidar_callback_simulation(self, msg: LaserScan):
        """Republish the LaserScan message adding frame id"""
        
        t = TransformStamped()

        # Read message content and assign it to corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map_ENU'
        t.child_frame_id = 'base_lidar'

        # The drone's attitude from the message (assuming q is [w, x, y, z])
        t.transform.rotation.w = float(self.vehicle_orientation.q[0])
        t.transform.rotation.x = float(self.vehicle_orientation.q[2])
        t.transform.rotation.y = float(self.vehicle_orientation.q[1])
        t.transform.rotation.z = -float(self.vehicle_orientation.q[3])

        t.transform.translation.x = self.vehicle_local_position.y
        t.transform.translation.y = self.vehicle_local_position.x
        t.transform.translation.z = -self.vehicle_local_position.z
        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


        # t1 = TransformStamped()
        # t1.header.stamp = self.get_clock().now().to_msg()
        # t1.header.frame_id = 'base_lidar_joint_angle'
        # t1.child_frame_id = 'base_lidar' # This depends joint angle
        # t1.transform.translation.x = 0.0
        # t1.transform.translation.y = 0.0
        # t1.transform.translation.z = 0.0
        # t1.transform.rotation.w = math.cos(self.actual_joint_angle / 2.0)
        # t1.transform.rotation.x = 0.0
        # t1.transform.rotation.y = 0.0
        # t1.transform.rotation.z = math.sin(self.actual_joint_angle / 2.0)
        # self.tf_broadcaster1.sendTransform(t1)

        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'base_lidar'
        t2.child_frame_id = 'base_laser'

        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.w = 0.70710678
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.70710678

        self.tf_broadcaster1.sendTransform(t2)

        msg.header.frame_id = 'base_laser'  # Ensure correct frame
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_simulation.publish(msg)

    def lidar_callback_hardware(self, msg: LaserScan):
        """Republish the LaserScan message adding frame id"""
        
        t = TransformStamped()

        # Read message content and assign it to corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map_ENU'
        t.child_frame_id = 'base_laser_aligned'
        # The drone's attitude from the message (assuming q is [w, x, y, z])
        t.transform.rotation.w = float(self.vehicle_orientation.q[0])
        t.transform.rotation.x = float(self.vehicle_orientation.q[2])
        t.transform.rotation.y = float(self.vehicle_orientation.q[1])
        t.transform.rotation.z = -float(self.vehicle_orientation.q[3])
        t.transform.translation.x = self.vehicle_local_position.y
        t.transform.translation.y = self.vehicle_local_position.x
        t.transform.translation.z = -self.vehicle_local_position.z
        self.tf_broadcaster.sendTransform(t)



        # t1 = TransformStamped()
        # t1.header.stamp = self.get_clock().now().to_msg()
        # t1.header.frame_id = 'base_laser_aligned_joint_angle'
        # t1.child_frame_id = 'base_laser_aligned' # This depends joint angle
        # t1.transform.translation.x = 0.0
        # t1.transform.translation.y = 0.0
        # t1.transform.translation.z = 0.0
        # t1.transform.rotation.w = math.cos(self.actual_joint_angle / 2.0)
        # t1.transform.rotation.x = 0.0
        # t1.transform.rotation.y = 0.0
        # t1.transform.rotation.z = math.sin(self.actual_joint_angle / 2.0)
        # self.tf_broadcaster1.sendTransform(t1)




        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'base_laser_aligned'
        t2.child_frame_id = 'base_laser' # This depends on lidar sensor mounting
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.w = 0.3007058
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.953717
        self.tf_broadcaster1.sendTransform(t2)



        msg.header.frame_id = 'base_laser'  # Ensure correct frame
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_hardware.publish(msg)

    def vehicle_orientation_callback(self, msg: FDOrientationNED):
        self.vehicle_orientation.q = msg.q

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def actual_joint_angle_callback(self,msg):
            # self.get_logger().info(f"Received actual joint angle: {msg.data}")
            self.actual_joint_angle = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = LidarDataForwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
