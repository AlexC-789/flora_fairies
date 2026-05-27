#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped  # ← changed from Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # publish to /cmd_standard so mux handles safety
        self.cmd_pub = self.create_publisher(
            TwistStamped, '/cmd_standard', 10)  # ← changed

        # publish position for twin_core and mux
        self.pos_pub = self.create_publisher(
            String, '/robot/position', 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.yaw = 0.0
        self.odom_received = False
        self.robot_position = (3, 1)  # match grid start tile
        self.robot_direction = 'NORTH'

        # hardcoded path for now — visits both plants
        self.path = [
            (3, 1),  # start (plant_02)
            (2, 1),
            (2, 2),
            (2, 3),
            (1, 3),  # plant_01
        ]

        self.get_logger().info('Navigation node started')
        self.get_logger().info('Waiting for odometry...')

        while rclpy.ok() and not self.odom_received:
            rclpy.spin_once(self)

        self.get_logger().info('Odometry received — starting route')
        self.follow_path()

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.odom_received = True

    def follow_path(self):
        self.get_logger().info(f'Following path: {self.path}')
        for next_tile in self.path[1:]:
            self.move_to_tile(next_tile)
        self.stop_robot()
        self.get_logger().info('Route complete!')

    def move_to_tile(self, next_tile):
        current_row, current_col = self.robot_position
        next_row, next_col = next_tile
        row_diff = next_row - current_row
        col_diff = next_col - current_col

        if row_diff == -1: desired_direction = 'NORTH'
        elif row_diff == 1: desired_direction = 'SOUTH'
        elif col_diff == 1: desired_direction = 'EAST'
        elif col_diff == -1: desired_direction = 'WEST'
        else:
            self.get_logger().error(f'Invalid move!')
            return

        self.turn_to_direction(desired_direction)
        self.move_forward()
        self.robot_position = next_tile
        self.publish_position()

    def publish_position(self):
        msg = String()
        msg.data = __import__('json').dumps({
            'row': self.robot_position[0],
            'col': self.robot_position[1],
            'direction': self.robot_direction
        })
        self.pos_pub.publish(msg)
        self.get_logger().info(f'Position: {self.robot_position}')

    def turn_to_direction(self, desired_direction):
        direction_angles = {
            'EAST': 0.0,
            'NORTH': math.pi / 2,
            'WEST': math.pi,
            'SOUTH': -math.pi / 2
        }
        target_yaw = direction_angles[desired_direction]

        while rclpy.ok():
            rclpy.spin_once(self)
            error = math.atan2(
                math.sin(target_yaw - self.yaw),
                math.cos(target_yaw - self.yaw)
            )
            if abs(error) < 0.05:
                break
            cmd = TwistStamped()  # ← changed
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'base_link'
            cmd.twist.angular.z = max(-1.0, min(1.0, 1.5 * error))
            self.cmd_pub.publish(cmd)
            time.sleep(0.02)

        self.stop_robot()
        self.robot_direction = desired_direction

    def move_forward(self):
        cmd = TwistStamped()  # ← changed
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = 0.15
        end_time = time.time() + 1.5
        while time.time() < end_time:
            cmd.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(cmd)
            time.sleep(0.05)
        self.stop_robot()

    def stop_robot(self):
        cmd = TwistStamped()  # ← changed
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        self.cmd_pub.publish(cmd)
        time.sleep(0.3)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()