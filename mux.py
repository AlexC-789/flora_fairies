#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class Mux(Node):
    def __init__(self):
        super().__init__('mux')

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.std_sub = self.create_subscription(
            TwistStamped, '/cmd_standard', self.standard_callback, 10)

        self.publisher_ = self.create_publisher(
            TwistStamped, '/cmd_vel', 10)
        self.obstacle_pub = self.create_publisher(
            Bool, '/obstacle/detected', 10)

        self.latest_standard_msg = TwistStamped()
        self.last_min_distance = 100.0
        self.safe_distance = 0.25
        self.was_blocked = False

        self.get_logger().info('Mux active')

    def lidar_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if r > 0.01 and r < 10.0]
        if valid_ranges:
            self.last_min_distance = min(valid_ranges)
        self.mux_check()

    def standard_callback(self, msg):
        self.latest_standard_msg = msg

    def mux_check(self):
        final_msg = TwistStamped()
        final_msg.header.stamp = self.get_clock().now().to_msg()
        final_msg.header.frame_id = 'base_link'

        if self.last_min_distance < self.safe_distance:
            final_msg.twist.linear.x = 0.0
            final_msg.twist.angular.z = 0.0
            self.get_logger().warn(
                f'Obstacle at {self.last_min_distance:.2f}m! Stopping.')
            # tell twin_core obstacle detected
            if not self.was_blocked:
                self.was_blocked = True
                obs = Bool()
                obs.data = True
                self.obstacle_pub.publish(obs)
        else:
            final_msg.twist.linear.x = \
                self.latest_standard_msg.twist.linear.x
            final_msg.twist.angular.z = \
                self.latest_standard_msg.twist.angular.z
            # tell twin_core obstacle cleared
            if self.was_blocked:
                self.was_blocked = False
                obs = Bool()
                obs.data = False
                self.obstacle_pub.publish(obs)

        self.publisher_.publish(final_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Mux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()