#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
import time
#hey
class RouteRunner(Node):
    def __init__(self):
        super().__init__('route_runner')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_standard', 10)
        
        # Subscribe to LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        
        self.last_min_distance = 100.0
        self.safe_distance = 0.25
        self.clear_count = 0          # ← counts consecutive clear readings
        self.CLEAR_THRESHOLD = 5      # ← must be clear 5 times in a row to resume

        self.get_logger().info('Route Runner Node has been started')

    def lidar_callback(self, msg):
        # Only check front 60 degrees (index 0-30 and 330-360)
        front_ranges = (
            list(msg.ranges[0:30]) + 
            list(msg.ranges[330:360])
        )
        valid = [r for r in front_ranges if r > 0.01 and r < 10.0]
        if valid:
            self.last_min_distance = min(valid)

    def is_blocked(self):
        return self.last_min_distance < self.safe_distance

    def wait_if_blocked(self):
        """Pauses route until obstacle is consistently gone"""
        if self.is_blocked():
            self.get_logger().warn(
                f'Obstacle at {self.last_min_distance:.2f}m — pausing...')
            self._send_stop()
            self.clear_count = 0

            # Wait until clear 5 times in a row
            while self.clear_count < self.CLEAR_THRESHOLD:
                rclpy.spin_once(self, timeout_sec=0.1)
                if not self.is_blocked():
                    self.clear_count += 1
                else:
                    self.clear_count = 0  # reset if blocked again

            self.get_logger().info('Consistently clear — resuming!')

    def move(self, linear=0.0, angular=0.0, duration=1.0):
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular

        end_time = time.time() + duration

        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.is_blocked():
                remaining = end_time - time.time()
                self.wait_if_blocked()
                # Resume with remaining time
                end_time = time.time() + remaining

            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(msg)
            time.sleep(0.1)

        self._send_stop()
        time.sleep(1)

    def _send_stop(self):
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        self.publisher_.publish(stop_msg)

    def run_route(self):
        self.get_logger().info('Starting route...')
        self.get_logger().info('Moving forward')
        self.move(linear=0.5, duration=10.0)
        self.get_logger().info('Turning left')
        self.move(angular=0.7, duration=5.0)
        self.get_logger().info('Moving forward')
        self.move(linear=0.5, duration=10.0)
        self.get_logger().info('Turning left')
        self.move(angular=0.7, duration=5.0)
        self.get_logger().info('Moving forward')
        self.move(linear=0.5, duration=10.0)
        self.get_logger().info('Route complete!')

def main(args=None):
    rclpy.init(args=args)
    node = RouteRunner()
    time.sleep(2.0)
    try:
        for i in range(5):
            node.get_logger().info(f'-~- Starting Lap {1+i} of 5 -~-')
            node.run_route()
            time.sleep(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        node._send_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()