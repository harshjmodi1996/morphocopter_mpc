import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Point

class SegmentVisualizer(Node):

    def __init__(self):
        super().__init__('segment_visualizer')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/folding_drone/in/obstacle_segments',
            self.callback,
            10)

        self.marker_pub = self.create_publisher(Marker, '/folding_drone/rviz_segments', 10)

        self.get_logger().info("Segment visualizer node started.")

    def callback(self, msg):
        if not msg.data:
            return

        # Extract layout
        dims = msg.layout.dim
        num_segments = dims[0].size if len(dims) > 0 else 0

        data = msg.data
        marker = Marker()
        marker.header.frame_id = "map_NED"  # or "odom" depending on your setup
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obstacle_segments"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.03  # line width (meters)
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker.points = []

        t = TransformStamped()

        # Read message content and assign it to corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map_ENU'
        t.child_frame_id = 'map_NED'

        # The transformation from ENU to NED is a fixed rotation.
        # It corresponds to a 90-degree rotation around Z, followed by a 180-degree rotation around the new X.
        # The resulting quaternion is (w=0, x=0.707, y=0.707, z=0).
        t.transform.rotation.w = 0.0
        t.transform.rotation.x = 0.7071067811865475
        t.transform.rotation.y = 0.7071067811865475
        t.transform.rotation.z = 0.0

        # No translation between the frames, their origins are the same.
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

        # Each segment = 2 points (x1,y1,z1) and (x2,y2,z2)
        # Flattened as [x1,y1,z1,x2,y2,z2,x1,y1,z1,x2,y2,z2,...]
        for i in range(0, len(data), 8):
            p1 = Point()
            p2 = Point()
            p1.x, p1.y, p1.z = data[i], data[i+1], data[i+2]
            p2.x, p2.y, p2.z = data[i+4], data[i+5], data[i+6]
            marker.points.append(p1)
            marker.points.append(p2)

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = SegmentVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
