#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class LaserScanToPointCloud(Node):
    def __init__(self) -> None:
        super().__init__("laserscan_to_pointcloud")
        self.declare_parameter("scan_topic", "/livox/lidar_scan")
        self.declare_parameter("cloud_topic", "/livox/lidar")
        self.declare_parameter("frame_id", "livox_lidar")

        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        cloud_topic = self.get_parameter("cloud_topic").get_parameter_value().string_value

        self._frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )

        self._pub = self.create_publisher(PointCloud2, cloud_topic, 10)
        self._sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, 10
        )
        self.get_logger().info(
            f"Converting LaserScan {scan_topic} -> PointCloud2 {cloud_topic}"
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        points: List[Tuple[float, float, float]] = []
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and r > 0.0:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0
                points.append((x, y, z))
            angle += msg.angle_increment

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        header = msg.header
        header.frame_id = self._frame_id or msg.header.frame_id
        cloud = point_cloud2.create_cloud(header, fields, points)
        self._pub.publish(cloud)


def main() -> None:
    rclpy.init()
    node = LaserScanToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
