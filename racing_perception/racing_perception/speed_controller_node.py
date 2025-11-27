import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist


class SpeedControllerNode(Node):
    def __init__(self):
        super().__init__('speed_controller_node')

        # Subscribe to safe speed from perception
        self.safe_speed_sub = self.create_subscription(
            Float32,
            '/racing/safe_speed',
            self.safe_speed_callback,
            10
        )

        # NEW: steering recommendation from perception
        self.steering_sub = self.create_subscription(
            Float32,
            '/racing/steering_cmd',
            self.steering_callback,
            10
        )

        # Publish commanded velocity
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Publish qualitative state
        self.state_pub = self.create_publisher(
            String,
            '/racing/state',
            10
        )

        # Internal state
        self.current_speed = 0.0
        self.target_speed = 0.0
        self.current_steering = 0.0
        self.steering_gain = 0.8   # scale steering into angular velocity

        self.max_accel = 0.5        # m/s^2
        self.control_period = 0.1   # 10 Hz

        # Timer for control loop
        self.timer = self.create_timer(
            self.control_period,
            self.control_step
        )

        self.get_logger().info('SpeedControllerNode with steering control started.')

    def safe_speed_callback(self, msg: Float32):
        self.target_speed = float(msg.data)

    def steering_callback(self, msg: Float32):
        self.current_steering = float(msg.data)

    def control_step(self):
        # Smooth acceleration/deceleration
        speed_error = self.target_speed - self.current_speed
        max_delta = self.max_accel * self.control_period

        if speed_error > max_delta:
            self.current_speed += max_delta
        elif speed_error < -max_delta:
            self.current_speed -= max_delta
        else:
            self.current_speed = self.target_speed

        # Build cmd_vel
        cmd = Twist()
        cmd.linear.x = self.current_speed

        # Steering behavior
        cmd.angular.z = self.current_steering * self.steering_gain

        # Send state label
        state_msg = String()
        if self.current_speed < 0.05:
            state_msg.data = 'STOP'
        elif self.current_speed < 1.0:
            state_msg.data = 'SLOW'
        else:
            state_msg.data = 'CRUISE'

        self.state_pub.publish(state_msg)
        self.cmd_vel_pub.publish(cmd)

        # Debug logs
        self.get_logger().debug(
            f"SpeedCmd={self.current_speed:.2f}, SteeringCmd={cmd.angular.z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SpeedControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
