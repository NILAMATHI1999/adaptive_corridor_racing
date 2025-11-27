import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node')

        # Create logs folder next to this file
        self.log_dir = os.path.join(os.path.dirname(__file__), 'logs')
        os.makedirs(self.log_dir, exist_ok=True)

        # Create a new CSV file name with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.log_dir, f'run_{timestamp}.csv')

        # Open CSV file
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)

        # Write header
        self.writer.writerow([
            'time',
            'corridor_width',
            'risk_level',
            'safe_speed',
            'steering_cmd',
            'behavior',
            'cmd_vel_linear',
            'cmd_vel_angular'
        ])

        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # Subscriptions
        self.create_subscription(Float32, '/racing/corridor_width', self.cb_corridor, 10)
        self.create_subscription(String,  '/racing/risk_level',     self.cb_risk,     10)
        self.create_subscription(Float32, '/racing/safe_speed',     self.cb_speed,    10)
        self.create_subscription(Float32, '/racing/steering_cmd',   self.cb_steer,    10)
        self.create_subscription(String,  '/racing/behavior',       self.cb_behavior, 10)
        self.create_subscription(Twist,   '/cmd_vel',               self.cb_cmdvel,   10)

        # Latest values
        self.corr = None
        self.risk = None
        self.safe_speed = None
        self.steering = None
        self.behavior = None
        self.lin = None
        self.ang = None

        # Timer to write one row every 0.1s (10 Hz)
        self.timer = self.create_timer(0.1, self.write_row)

        self.get_logger().info(f"Logging to: {self.csv_path}")

    # Callbacks to store latest values
    def cb_corridor(self, msg): self.corr = msg.data
    def cb_risk(self, msg): self.risk = msg.data
    def cb_speed(self, msg): self.safe_speed = msg.data
    def cb_steer(self, msg): self.steering = msg.data
    def cb_behavior(self, msg): self.behavior = msg.data

    def cb_cmdvel(self, msg: Twist):
        self.lin = msg.linear.x
        self.ang = msg.angular.z

    def write_row(self):
        now = self.get_clock().now().nanoseconds / 1e9
        t = now - self.start_time

        # Write one row even if some values are None
        self.writer.writerow([
            t,
            self.corr,
            self.risk,
            self.safe_speed,
            self.steering,
            self.behavior,
            self.lin,
            self.ang
        ])

    def destroy_node(self):
        # Close file cleanly on shutdown
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
