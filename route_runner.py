#!/usr/bin/env python3

#ROS client library 
import rclpy
#Node is base class for everything in ROS2 (every robot component should be a node)
from rclpy.node import Node
#Package of standard message types for describing movement and geometry -> TwistStamped contain a header (timestamp & frame) and twist (linear speed & angluar speed)
from geometry_msgs.msg import TwistStamped  # ← changed
#used for sleep() to control how long the robot moves
import time

#Class inherits from Node (routerunner is a ROS2 node, get built in abilites)
class RouteRunner(Node):
    def __init__(self):
        super().__init__('route_runner')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_standard', 10)  # Type of message, topic name, queue size
        self.get_logger().info('Route Runner Node has been started') #instead of print use get_logger

    def move(self, linear=0.0, angular=0.0, duration=1.0):
        msg = TwistStamped()  # changed, this will create a data container defined by ROS2 -> sends to cmd hardware controllers WHEN
        msg.header.frame_id = 'base_link' #coordiante frame context so robot knows which direction forward is relative to its body  
        msg.twist.linear.x = linear        #changed (now msg.twist.linear), forward/backward
        msg.twist.angular.z = angular      #changed (now msg.twist.angular), turning

        end_time = time.time() + duration

        while time.time() < end_time:
            msg.header.stamp = self.get_clock().now().to_msg()  #update timestamp each loop
            self.publisher_.publish(msg)
            time.sleep(0.1)

        # Stop (Publish to standard topic so Mux knows we want to stay still)
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        self.publisher_.publish(stop_msg)
        time.sleep(1)

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
        for i in range(5): #run 5 times
            node.get_logger().info(f'-~- Starting Lap {1+i} of 5 -~-')
            node.run_route()
            time.sleep(2.0)        
    except KeyboardInterrupt:
        pass #Maybe add emergency stop if user presses key x? 
    finally:
        #node.get_logger().info('All 5 laps have been completed succesfully :)')
        
        stop_msg = TwistStamped()
        stop_msg.header.stamp = node.get_clock().now().to_msg()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()