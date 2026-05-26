import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
import time

class TwinCore(Node):
    def __init__(self):
        self.grid = [] #from grid_layout.yaml
        self.blocked_tiles = set() #blocked by obstacles

        #robot tracking
        self.robot_pos = None #curr robot pos (row, col)
        self.robot_status = None 
        self.planned_route = [] #full path 

        #decision tracking
        self.currently_blocked_tile = None
        self.alternative_exists = True
        