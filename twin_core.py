#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
import json
import heapq

class TwinCore(Node):
    def __init__(self):
        super().__init__('twin_core')

        # state
        self.grid = None
        self.tile_size = 0.25
        self.robot_pos = None
        self.robot_grid_pos = None
        self.obstacle_active = False
        self.plant_measurements = {}

        # ── SUBSCRIBERS ──────────────────────────
        # get grid updates
        self.create_subscription(
            String, '/grid/state', self.on_grid, 10)

        # get real robot position from odom
        self.create_subscription(
            Odometry, '/odom', self.on_odom, 10)

        # get obstacle events from mux
        self.create_subscription(
            Bool, '/obstacle/detected', self.on_obstacle, 10)

        # get plant measurements
        self.create_subscription(
            String, '/plant/measurement', self.on_measurement, 10)

        # get robot position from navigation_node
        self.create_subscription(
            String, '/robot/position', self.on_robot_position, 10)

        # ── PUBLISHERS ───────────────────────────
        # send new path to navigation_node
        self.path_pub = self.create_publisher(
            String, '/twin/new_path', 10)

        # send resume after obstacle cleared
        self.resume_pub = self.create_publisher(
            Bool, '/twin/resume_granted', 10)

        # publish full twin state for visualiser
        self.state_pub = self.create_publisher(
            String, '/twin/state', 10)

        self.get_logger().info('Twin core active')

    def on_grid(self, msg):
        data = json.loads(msg.data)
        self.grid = data['grid']
        self.tile_size = data.get('tile_size', 0.25)
        self.get_logger().info('Grid received')
        self.publish_state()

    def on_odom(self, msg):
        # real position in meters from odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_pos = (x, y)

    def on_robot_position(self, msg):
        # grid position from navigation_node
        data = json.loads(msg.data)
        self.robot_grid_pos = (data['row'], data['col'])
        self.get_logger().info(
            f'Robot at tile ({data["row"]},{data["col"]})')
        self.publish_state()

    def on_obstacle(self, msg):
        self.obstacle_active = msg.data
        if msg.data:
            self.get_logger().warn('Obstacle detected — twin aware')
            # try to find alternative path
            self.handle_obstacle()
        else:
            self.get_logger().info('Obstacle cleared — resuming')
            resume = Bool()
            resume.data = True
            self.resume_pub.publish(resume)
        self.publish_state()

    def on_measurement(self, msg):
        data = json.loads(msg.data)
        plant_id = data['plant_id']
        temp = data['temp']
        self.plant_measurements[plant_id] = temp
        self.get_logger().info(
            f'Twin received: {plant_id} = {temp}°C')
        self.publish_state()

    def handle_obstacle(self):
        # when obstacle detected try to find alternative
        if self.grid is None or self.robot_grid_pos is None:
            return

        # find all unvisited plants
        waypoints = self.find_waypoints()
        if not waypoints:
            return

        # try A* to next waypoint with current grid
        # (obstacle_guard already marked blocked tile as 1)
        start = self.robot_grid_pos
        goal = waypoints[0]
        path = self.astar(self.grid, start, goal)

        if path:
            # alternative path exists — send it
            self.get_logger().info('Alternative path found — rerouting')
            msg = String()
            msg.data = json.dumps({'path': path})
            self.path_pub.publish(msg)
        else:
            # no alternative — tell robot to wait
            self.get_logger().warn('No alternative path — waiting')
            resume = Bool()
            resume.data = False
            self.resume_pub.publish(resume)

    def find_waypoints(self):
        if self.grid is None:
            return []
        waypoints = []
        for row in range(len(self.grid)):
            for col in range(len(self.grid[row])):
                if self.grid[row][col] == 2:
                    waypoints.append((row, col))
        return waypoints

    def publish_state(self):
        state = {
            'robot_grid_pos': self.robot_grid_pos,
            'obstacle_active': self.obstacle_active,
            'plant_measurements': self.plant_measurements,
            'grid': self.grid
        }
        msg = String()
        msg.data = json.dumps(state)
        self.state_pub.publish(msg)

    def astar(self, grid, start, goal):
        open_set = []
        closed_set = set()
        came_from = {}
        g_score = {start: 0}
        heapq.heappush(open_set, (self.heuristic(start, goal), start))

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]
            closed_set.add(current)
            row, col = current
            for dr, dc in [(0,1),(1,0),(0,-1),(-1,0)]:
                nr, nc = row+dr, col+dc
                neighbor = (nr, nc)
                if neighbor in closed_set:
                    continue
                if not (0 <= nr < len(grid) and 0 <= nc < len(grid[0])):
                    continue
                if grid[nr][nc] == 1:
                    continue
                tg = g_score[current] + 1
                if neighbor not in g_score or tg < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tg
                    f = tg + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))
        return None

    def heuristic(self, a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

def main(args=None):
    rclpy.init(args=args)
    node = TwinCore()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()