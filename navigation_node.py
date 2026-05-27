#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion


class NavigationNode(Node):

    def __init__(self):

        super().__init__('navigation_node')

        # ==================================================
        # Publisher
        # ==================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # ==================================================
        # Odometry Subscriber
        # ==================================================

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # ==================================================
        # Robot State
        # ==================================================

        # Robot yaw (rotation)
        self.yaw = 0.0

        # Have we received odom yet?
        self.odom_received = False

        # Current robot grid position
        self.robot_position = (4, 0)

        # Current direction robot is facing
        self.robot_direction = 'NORTH'

        # ==================================================
        # Example Path From A*
        # ==================================================

        self.path = [
            (4, 0),
            (3, 0),
            (2, 0),
            (2, 1),
            (2, 2),
        ]

        self.get_logger().info('Navigation node started')

        # ==================================================
        # Wait for odometry before moving
        # ==================================================

        self.get_logger().info('Waiting for odometry...')

        while rclpy.ok() and not self.odom_received:
            rclpy.spin_once(self)

        self.get_logger().info('Odometry received')

        # ==================================================
        # Start navigation
        # ==================================================

        self.follow_path()

    # ==================================================
    # Odometry Callback
    # ==================================================

    def odom_callback(self, msg):

        q = msg.pose.pose.orientation

        quaternion = (
            q.x,
            q.y,
            q.z,
            q.w
        )

        # Convert quaternion -> yaw
        _, _, self.yaw = euler_from_quaternion(quaternion)

        self.odom_received = True

    # ==================================================
    # Follow Full Path
    # ==================================================

    def follow_path(self):

        self.get_logger().info(
            f'Following path: {self.path}'
        )

        for next_tile in self.path[1:]:

            self.move_to_tile(next_tile)

        self.stop_robot()

        self.get_logger().info('Goal reached')

    # ==================================================
    # Move To Tile
    # ==================================================

    def move_to_tile(self, next_tile):

        current_row, current_col = self.robot_position

        next_row, next_col = next_tile

        # ------------------------------------------
        # Determine movement direction
        # ------------------------------------------

        row_diff = next_row - current_row
        col_diff = next_col - current_col

        if row_diff == -1:
            desired_direction = 'NORTH'

        elif row_diff == 1:
            desired_direction = 'SOUTH'

        elif col_diff == 1:
            desired_direction = 'EAST'

        elif col_diff == -1:
            desired_direction = 'WEST'

        else:
            self.get_logger().error(
                f'Invalid move from {self.robot_position} to {next_tile}'
            )
            return

        self.get_logger().info(
            f'Moving from {self.robot_position} to {next_tile}'
        )

        # ------------------------------------------
        # Turn robot
        # ------------------------------------------

        self.turn_to_direction(desired_direction)

        # ------------------------------------------
        # Move forward one tile
        # ------------------------------------------

        self.move_forward()

        # ------------------------------------------
        # Update grid position
        # ------------------------------------------

        self.robot_position = next_tile

        self.get_logger().info(
            f'Robot position updated: {self.robot_position}'
        )

    # ==================================================
    # Turn Robot To Desired Direction
    # ==================================================

    def turn_to_direction(self, desired_direction):

        # Map directions to angles
        direction_angles = {
            'EAST': 0.0,
            'NORTH': math.pi / 2,
            'WEST': math.pi,
            'SOUTH': -math.pi / 2
        }

        target_yaw = direction_angles[desired_direction]

        self.get_logger().info(
            f'Turning toward {desired_direction}'
        )

        while rclpy.ok():

            # Process odom updates
            rclpy.spin_once(self)

            # ------------------------------------------
            # Calculate angle error
            # ------------------------------------------

            error = target_yaw - self.yaw

            # Normalize angle
            error = math.atan2(
                math.sin(error),
                math.cos(error)
            )

            # ------------------------------------------
            # Stop if aligned
            # ------------------------------------------

            if abs(error) < 0.05:
                break

            # ------------------------------------------
            # Rotate robot
            # ------------------------------------------

            cmd = Twist()

            # Proportional controller
            cmd.angular.z = 1.5 * error

            # Clamp rotation speed
            if cmd.angular.z > 1.0:
                cmd.angular.z = 1.0

            if cmd.angular.z < -1.0:
                cmd.angular.z = -1.0

            self.cmd_pub.publish(cmd)

            time.sleep(0.02)

        self.stop_robot()

        # Update stored direction
        self.robot_direction = desired_direction

        self.get_logger().info(
            f'Now facing {desired_direction}'
        )

    # ==================================================
    # Move Forward One Tile
    # ==================================================

    def move_forward(self):

        self.get_logger().info(
            'Moving forward one tile'
        )

        cmd = Twist()

        cmd.linear.x = 0.15

        self.cmd_pub.publish(cmd)

        # Move for fixed time
        time.sleep(1.5)

        self.stop_robot()

    # ==================================================
    # Stop Robot
    # ==================================================

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


# ======================================================
# Main
# ======================================================

def main(args=None):

    rclpy.init(args=args)

    node = NavigationNode()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()