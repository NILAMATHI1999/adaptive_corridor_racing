import math
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FakeLidarNode(Node):
    def __init__(self):
        super().__init__('fake_lidar_node')

        # Publisher to /scan (same topic your perception node listens to)
        self.lidar_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        # LaserScan configuration
        self.angle_min = -math.radians(90.0)   # -90 degrees
        self.angle_max = math.radians(90.0)    # +90 degrees
        self.num_points = 360
        self.angle_increment = (self.angle_max - self.angle_min) / self.num_points
        self.range_min = 0.1
        self.range_max = 10.0

        # Step counter to change scenario over time
        self.step = 0

        self.get_logger().info('FakeLidarNode started, publishing /scan with varying corridor.')

    def timer_callback(self):
        self.step += 1

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'

        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        # Base: open space
        ranges: List[float] = [5.0] * self.num_points
        center = self.num_points // 2

        # Always: front obstacle at 0.8 m
        for i in range(center - 3, center + 4):
            ranges[i] = 0.8

        # Change corridor over time in 3 phases:
        # 0–100 steps: SAFE (wide corridor)
        # 100–200 steps: NARROW
        # 200+ steps: CRITICAL
        phase = (self.step // 100) % 3

        if phase == 0:
            # SAFE: left and right relatively far
            right_dist = 5.0
            left_dist = 5.0
        elif phase == 1:
            # NARROW: left and right closer
            right_dist = 1.0
            left_dist = 1.0
        else:
            # CRITICAL: very tight corridor
            right_dist = 0.4
            left_dist = 0.4

        # RIGHT side sector
        for i in range(center - 100, center - 40):
            ranges[i] = right_dist

        # LEFT side sector
        for i in range(center + 40, center + 100):
            ranges[i] = left_dist

        msg.ranges = ranges
        self.lidar_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
