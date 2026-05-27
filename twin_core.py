#!/usr/bin/env python3
"""
Minimal Twin Core for Demo

Does ONE thing:
1. Reads grid from grid_node
2. Reads robot position 
3. Plans path to goal using A*
4. Publishes path

That's it. No bridges, no fancy stuff.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json
import numpy as np

from my_tb3_world.astar import astar


class TwinCore(Node):
    def __init__(self):
        super().__init__('twin_core')
        
        # State
        self.grid = None
        self.robot_pos = None
        self.goal = (3.0, 3.0)  # Default goal in meters (modify as needed)
        
        # Subscriptions
        self.create_subscription(
            String,
            '/grid/state',
            self.on_grid,
            10
        )
        
        self.create_subscription(
            Odometry,
            '/odom',
            self.on_odom,
            10
        )
        
        # Publisher
        self.path_pub = self.create_publisher(
            Path,
            '/plan/path',
            10
        )
        
        # Plan every time grid updates
        self.get_logger().info("Twin Core ready")

    def on_grid(self, msg: String):
        """Grid updated - parse and replan"""
        try:
            data = json.loads(msg.data)
            self.grid = data['grid']
            tile_size = data.get('tile_size', 0.25)
            
            self.get_logger().debug(f"Grid received: {len(self.grid)}x{len(self.grid[0])}")
            
            # Replan immediately
            self.plan(tile_size)
            
        except Exception as e:
            self.get_logger().error(f"Grid parse failed: {e}")

    def on_odom(self, msg: Odometry):
        """Robot moved - store position"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_pos = (x, y)
        self.get_logger().debug(f"Robot at ({x:.2f}, {y:.2f})")

    def plan(self, tile_size):
        """Run A* and publish path"""
        if self.robot_pos is None:
            self.get_logger().warn("No robot position yet")
            return
        
        if self.grid is None:
            self.get_logger().warn("No grid yet")
            return
        
        # Convert world coords to grid coords
        start_row = int(self.robot_pos[1] / tile_size)
        start_col = int(self.robot_pos[0] / tile_size)
        goal_row = int(self.goal[1] / tile_size)
        goal_col = int(self.goal[0] / tile_size)
        
        # Bounds check
        grid_height = len(self.grid)
        grid_width = len(self.grid[0]) if grid_height > 0 else 0
        
        if not (0 <= start_row < grid_height and 0 <= start_col < grid_width):
            self.get_logger().error(f"Start out of bounds: ({start_row}, {start_col})")
            return
        
        if not (0 <= goal_row < grid_height and 0 <= goal_col < grid_width):
            self.get_logger().error(f"Goal out of bounds: ({goal_row}, {goal_col})")
            return
        
        self.get_logger().info(
            f"Planning from ({start_row}, {start_col}) to ({goal_row}, {goal_col})"
        )
        
        # Run A*
        path_grid = astar(self.grid, (start_row, start_col), (goal_row, goal_col))
        
        if path_grid is None:
            self.get_logger().warn("No path found!")
            return
        
        # Convert path back to world coords
        path_world = []
        for row, col in path_grid:
            x = col * tile_size + tile_size / 2
            y = row * tile_size + tile_size / 2
            path_world.append((x, y))
        
        # Publish path
        self.publish_path(path_world)
        self.get_logger().info(f"Path published: {len(path_world)} waypoints")

    def publish_path(self, waypoints):
        """Publish as nav_msgs/Path"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwinCore()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()