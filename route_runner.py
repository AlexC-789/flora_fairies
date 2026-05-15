#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped  # ← changed
import time

class RouteRunner(Node):
    def __init__(self):
        super().__init__('route_runner')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)  # ← changed
        self.get_logger().info('Route Runner Node has been started')

    def move(self, linear=0.0, angular=0.0, duration=1.0):
        msg = TwistStamped()  # ← changed
        msg.header.frame_id = 'base_link'  # ← new
        msg.twist.linear.x = linear        # ← changed (now msg.twist.linear)
        msg.twist.angular.z = angular      # ← changed (now msg.twist.angular)

        end_time = time.time() + duration
        while time.time() < end_time:
            msg.header.stamp = self.get_clock().now().to_msg()  # ← update timestamp each loop
            self.publisher_.publish(msg)
            time.sleep(0.1)

        # Stop
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        self.publisher_.publish(stop_msg)
        time.sleep(1)

    def run_route(self):
        self.get_logger().info('Starting route...')
        self.get_logger().info('Moving forward')
        self.move(linear=0.2, duration=5.0)
        self.get_logger().info('Turning left')
        self.move(angular=0.5, duration=3.0)
        self.get_logger().info('Moving forward')
        self.move(linear=0.2, duration=5.0)
        self.get_logger().info('Turning left')
        self.move(angular=0.5, duration=3.0)
        self.get_logger().info('Moving forward')
        self.move(linear=0.2, duration=5.0)
        self.get_logger().info('Route complete!')

def main(args=None):
    rclpy.init(args=args)
    node = RouteRunner()
    time.sleep(2.0)
    try:
        node.run_route()
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = TwistStamped()
        stop_msg.header.stamp = node.get_clock().now().to_msg()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()