import rclpy #ROS 2 for python
from rclpy.node import Node 
from std_msgs.msg import String # to send text data between nodes
import json #to convert between python dictionaries and text
import random #to generate random numbers

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        # The publisher sends measurements to plant_monitor
        self.measurement_pub = self.create_publisher(
            String,
            '/plant/measurement',
            10 #buffer size
        )

        # The subscriber receives correction commands from plant_monitor
        self.correction_sub = self.create_subscription(
            String,
            '/twin/correction_cmd',
            self.correction_callback,
            10 #buffer
        )

        self.get_logger().info('Sensor node started')

    def take_measurement(self, plant_id, row, col):
        # Generates a fake temperature between 18 and 26 degrees.
        temp = round(random.uniform(18.0, 26.0), 1)

        # Packages it as JSON and publishes it
        data = {
            'plant_id': plant_id,
            'temp': temp,
            'row': row,
            'col': col
        }
        msg = String()
        msg.data = json.dumps(data)
        self.measurement_pub.publish(msg)

        self.get_logger().info(f'Plant {plant_id} at ({row},{col}): {temp}°C')
        return temp

    def correction_callback(self, msg):
        # This fires when plant_monitor sends a correction
        data = json.loads(msg.data)
        self.get_logger().info(f'Correction received for plant {data["plant_id"]}: {data["correction"]}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#sources: 
#publusher/subscriber: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#Creating a json object: https://www.w3schools.com/python/python_json.asp