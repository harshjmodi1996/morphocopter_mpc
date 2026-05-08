#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
import yaml
import numpy as np
import cv2
import os
from builtin_interfaces.msg import Time

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        # Path to your YAML file
        yaml_path = "/home/harshmodi/PX4-Autopilot/Tools/simulation/gz/models/realistic_room/map.yaml"
        self.get_logger().info(f"Loading map YAML: {yaml_path}")

        if not os.path.exists(yaml_path):
            self.get_logger().error("YAML file not found!")
            exit(1)

        # Load YAML
        with open(yaml_path, 'r') as file:
            self.map_info = yaml.safe_load(file)

        # Load image
        image_path = os.path.join(os.path.dirname(yaml_path), self.map_info["image"])
        self.get_logger().info(f"Loading map image: {image_path}")

        if not os.path.exists(image_path):
            self.get_logger().error("Map image file not found!")
            exit(1)

        # Load as grayscale
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error("Failed to load map image!")
            exit(1)

        # Flip the image vertically to match the map coordinate system
        img = np.flipud(img)

        # Convert image to occupancy format
        # (PGM: 0=occupied, 255=free)
        occ = np.zeros_like(img, dtype=np.int8)
        occ[img == 0] = 100       # occupied
        occ[img == 255] = 0       # free
        occ[(img != 0) & (img != 255)] = -1  # unknown

        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.header.frame_id = "map_ENU"

        # Fill metadata
        meta = MapMetaData()
        meta.resolution = float(self.map_info["resolution"])
        meta.width = img.shape[1]
        meta.height = img.shape[0]
        meta.origin.position.x = float(self.map_info["origin"][0])
        meta.origin.position.y = float(self.map_info["origin"][1])
        meta.origin.position.z = float(self.map_info["origin"][2])

        self.occupancy_grid.info = meta
        self.occupancy_grid.data = occ.flatten().tolist()

        self.publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(0.5, self.publish_map)

        self.get_logger().info("Map publisher initialized!")

    def publish_map(self):
        self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.occupancy_grid)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
