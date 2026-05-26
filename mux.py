#!/usr/bin/env python3

#ROS client library 
import rclpy
#Node is base class for everything in ROS2 (every robot component should be a node)
from rclpy.node import Node
#Package of standard message types for describing movement and geometry -> TwistStamped contain a header (timestamp & frame) and twist (linear speed & angluar speed)
from geometry_msgs.msg import TwistStamped  # ← changed
from sensor_msgs.msg import LaserScan
#used for sleep() to control how long the robot moves
import time

#Class inherits from Node (is a ROS2 node, get built in abilites)
class Mux(Node):
    def __init__(self):
        super().__init__('mux')
        
        #get scan info -> oh no 'obstacle'
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        
        # follows instruction of route
        self.std_sub = self.create_subscription(TwistStamped, '/cmd_standard', self.standard_callback, 10)

        # tells robot what to do -> /cmd_vel is always robo
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10) 

        # Variables (safe)
        self.latest_standard_msg = TwistStamped()
        self.last_min_distance = 100.0  
        self.safe_distance = 0.25

        self.get_logger().info('Mux is active and keeping Pixie Planter from bumping accidents')

    def lidar_callback(self, msg):
        # Filter out 0.0 and inf values to get real distances
        valid_ranges = [r for r in msg.ranges if r > 0.01 and r < 10.0]
        if valid_ranges:
            self.last_min_distance = min(valid_ranges)
        self.mux_check()

    def standard_callback(self, msg):
        # Store what RouteRunner wants to do
        self.latest_standard_msg = msg

    def mux_check(self):
        final_msg = TwistStamped()
        final_msg.header.stamp = self.get_clock().now().to_msg()
        final_msg.header.frame_id = 'base_link'

        if self.last_min_distance < self.safe_distance:
            # EMERGENCY STOP
            final_msg.twist.linear.x = 0.0
            final_msg.twist.angular.z = 0.0
            self.get_logger().warn(f"Obstacle at {self.last_min_distance:.2f}m! Stopping.")
        else:
            # ALL CLEAR: Forward Route Runner data
            final_msg.twist.linear.x = self.latest_standard_msg.twist.linear.x
            final_msg.twist.angular.z = self.latest_standard_msg.twist.angular.z

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