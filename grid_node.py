#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

# Import your custom service types
# my_tb3_world = package name
# srv = folder
# UpdateTile/GetGrid = filename without .srv
from my_tb3_world.srv import UpdateTile
from my_tb3_world.srv import GetGrid

class Grid_node(Node):
    def __init__(self):
        super().__init__('grid_node')
        
        self.grid = [
            [1, 1, 1, 1, 1],
            [1, 0, 0, 2, 1],
            [1, 0, 0, 0, 1],
            [1, 2, 0, 0, 1],
            [1, 1, 1, 1, 1],
        ]
        self.tile_size = 0.25

        # Publisher
        self.grid_pub = self.create_publisher(
            String, '/grid/state', 10)

        # Services using your custom types
        self.update_srv = self.create_service(
            UpdateTile,              # ← your custom type
            '/grid/update_tile',
            self.handle_update_tile
        )

        self.get_grid_srv = self.create_service(
            GetGrid,                 # ← your custom type
            '/grid/get_grid',
            self.handle_get_grid
        )

        self.publish_grid()
        self.get_logger().info('Grid node active')

    def publish_grid(self):
        msg = String()
        msg.data = json.dumps({
            'grid': self.grid,
            'tile_size': self.tile_size
        })
        self.grid_pub.publish(msg)

    def handle_update_tile(self, request, response):
        # request.row, request.col, request.value
        # come directly from the .srv definition!
        row = request.row
        col = request.col
        value = request.value

        # safety checks
        if row < 0 or row >= len(self.grid):
            response.success = False
            response.message = 'Row out of bounds'
            return response

        if col < 0 or col >= len(self.grid[0]):
            response.success = False
            response.message = 'Col out of bounds'
            return response

        # never overwrite walls or plants
        if self.grid[row][col] == 1 or self.grid[row][col] == 2:
            response.success = False
            response.message = f'Cannot overwrite tile type {self.grid[row][col]}'
            return response

        # update the tile
        self.grid[row][col] = value
        self.get_logger().info(f'Tile ({row},{col}) updated to {value}')

        # tell everyone the grid changed
        self.publish_grid()

        response.success = True
        response.message = f'Tile ({row},{col}) set to {value}'
        return response

    def handle_get_grid(self, request, response):
        # request is empty (as defined in GetGrid.srv)
        # just return the grid
        response.grid_data = json.dumps(self.grid)
        response.tile_size = self.tile_size
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Grid_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()