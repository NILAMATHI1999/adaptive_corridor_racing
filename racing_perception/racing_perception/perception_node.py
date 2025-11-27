import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String


class RacingPerceptionNode(Node):
    def __init__(self):
        super().__init__('racing_perception_node')

        # Subscribe to LiDAR scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # Publishers
        self.speed_pub = self.create_publisher(Float32, '/racing/safe_speed', 10)
        self.corridor_pub = self.create_publisher(Float32, '/racing/corridor_width', 10)
        self.risk_pub = self.create_publisher(String, '/racing/risk_level', 10)
        self.steering_pub = self.create_publisher(Float32, '/racing/steering_cmd', 10)
        self.behavior_pub = self.create_publisher(String, '/racing/behavior', 10)

        # Parameters
        self.front_angle_width_deg = 60.0   # +/- 30 deg
        self.right_min_deg = -70.0
        self.right_max_deg = -10.0
        self.left_min_deg = 10.0
        self.left_max_deg = 70.0

        self.min_distance_stop = 0.5
        self.min_distance_slow = 2.0

        self.corridor_narrow = 3.0      # corridor < 3 m → NARROW
        self.corridor_critical = 1.0    # corridor < 1 m → CRITICAL

        self.get_logger().info('FULL RacingPerceptionNode started (speed + steering + behavior).')

    # Helper: convert degree range to index range
    def get_indices(self, msg, deg_min, deg_max):
        rad_min = math.radians(deg_min)
        rad_max = math.radians(deg_max)

        idx_min = None
        idx_max = None

        for i in range(len(msg.ranges)):
            ang = msg.angle_min + i * msg.angle_increment

            if idx_min is None and ang >= rad_min:
                idx_min = i

            if ang <= rad_max:
                idx_max = i

        if idx_min is None:
            idx_min = 0
        if idx_max is None:
            idx_max = len(msg.ranges) - 1

        return idx_min, idx_max

    # MAIN CALLBACK
    def scan_callback(self, msg: LaserScan):

        ranges = msg.ranges
        n = len(ranges)

        # -------------- FRONT SECTOR --------------
        half_front = math.radians(self.front_angle_width_deg / 2)
        front_dists = [
            ranges[i] for i in range(n)
            if -half_front <= (msg.angle_min + i * msg.angle_increment) <= half_front
            and not math.isinf(ranges[i]) and not math.isnan(ranges[i])
        ]

        min_front_dist = min(front_dists) if front_dists else msg.range_max

        # -------------- LEFT & RIGHT SECTORS --------------
        right_i_min, right_i_max = self.get_indices(msg, self.right_min_deg, self.right_max_deg)
        left_i_min, left_i_max = self.get_indices(msg, self.left_min_deg, self.left_max_deg)

        right_vals = [
            ranges[i] for i in range(right_i_min, right_i_max + 1)
            if not math.isinf(ranges[i]) and not math.isnan(ranges[i])
        ]
        left_vals = [
            ranges[i] for i in range(left_i_min, left_i_max + 1)
            if not math.isinf(ranges[i]) and not math.isnan(ranges[i])
        ]

        min_right = min(right_vals) if right_vals else msg.range_max
        min_left = min(left_vals) if left_vals else msg.range_max

        # -------------- CORRIDOR WIDTH --------------
        corridor_width = float(min_left + min_right)

        risk_msg = String()
        if corridor_width < self.corridor_critical:
            risk_msg.data = 'CRITICAL'
        elif corridor_width < self.corridor_narrow:
            risk_msg.data = 'NARROW'
        else:
            risk_msg.data = 'SAFE'

        # -------------- SAFE SPEED --------------
        if min_front_dist < self.min_distance_stop:
            base_speed = 0.0
        elif min_front_dist < self.min_distance_slow:
            ratio = (min_front_dist - self.min_distance_stop) / (
                self.min_distance_slow - self.min_distance_stop
            )
            base_speed = 0.5 + 0.5 * ratio
        else:
            base_speed = 2.0

        # Corridor factor
        if risk_msg.data == 'SAFE':
            corridor_factor = 1.0
        elif risk_msg.data == 'NARROW':
            corridor_factor = 0.7
        else:
            corridor_factor = 0.3

        recommended_speed = base_speed * corridor_factor

        # -------------- STEERING DECISION --------------
        steering = 0.0
        if risk_msg.data != 'CRITICAL':
            if min_left - min_right > 0.2:
                steering = -1.0  # steer left
            elif min_right - min_left > 0.2:
                steering = 1.0   # steer right
            else:
                steering = 0.0

        # -------------- BEHAVIOR PLANNING --------------
        behavior_msg = String()
        if risk_msg.data == 'CRITICAL':
            behavior_msg.data = 'STOP'
        elif risk_msg.data == 'NARROW':
            if min_left > min_right:
                behavior_msg.data = 'SHIFT_LEFT'
            else:
                behavior_msg.data = 'SHIFT_RIGHT'
        else:
            behavior_msg.data = 'CRUISE'

        # -------------- PUBLISH EVERYTHING --------------
        speed_msg = Float32()
        speed_msg.data = recommended_speed
        self.speed_pub.publish(speed_msg)

        corridor_msg = Float32()
        corridor_msg.data = corridor_width
        self.corridor_pub.publish(corridor_msg)

        self.risk_pub.publish(risk_msg)

        steering_msg = Float32()
        steering_msg.data = steering
        self.steering_pub.publish(steering_msg)

        self.behavior_pub.publish(behavior_msg)

        # -------------- LOG --------------
        self.get_logger().info(
            f"Front={min_front_dist:.2f} | L={min_left:.2f} R={min_right:.2f} "
            f"| Corr={corridor_width:.2f} ({risk_msg.data}) "
            f"| Speed={recommended_speed:.2f} | Steer={steering} "
            f"| Behavior={behavior_msg.data}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RacingPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
