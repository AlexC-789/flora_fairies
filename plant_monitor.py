import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class PlantMonitor(Node):
    
    def __init__(self):
        super().__init__('plant_monitor')

        # Each plant's optimal temperature. We have to change this if person3 named them differently!!!.
        self.optimal_temps = {
            'plant_01': 22.0,
            'plant_02': 21.0,
            'plant_03': 23.0,
            'plant_04': 22.0,
        }

        # Logs of all measurements per plant
        self.plant_logs = {}

        # The subscriber listens for measurements from sensor_node
        self.measurement_sub = self.create_subscription(
            String,
            '/plant/measurement',
            self.measurement_callback,
            10
        )

        # The publisher sends corrections back to sensor_node
        self.correction_pub = self.create_publisher(
            String,
            '/twin/correction_cmd',
            10
        )

        self.get_logger().info('Plant monitor started')

    def measurement_callback(self, msg):
        # This fires every time sensor_node publishes a measurement
        data = json.loads(msg.data)

        plant_id = data['plant_id']
        temp = data['temp']

        # Logs the measurement
        if plant_id not in self.plant_logs:
            self.plant_logs[plant_id] = []
        self.plant_logs[plant_id].append(temp)

        self.get_logger().info(f'Received: Plant {plant_id} = {temp}°C')

        # Checks if temperature is within +-2 of optimal
        self.check_health(plant_id, temp)

    def check_health(self, plant_id, temp):
        if plant_id not in self.optimal_temps:
            self.get_logger().warn(f'Unknown plant: {plant_id}')
            return

        optimal = self.optimal_temps[plant_id]
        difference = abs(temp - optimal)

        if difference <= 2.0:
            # Temperature is fine
            self.get_logger().info(f'Plant {plant_id}: {temp}°C healthy')
        else:
            # Temperature is outside range. Send correction
            self.get_logger().warn(f'Plant {plant_id}: {temp}°C outside range!')
            self.send_correction(plant_id, temp, optimal)

    def send_correction(self, plant_id, temp, optimal):
        # Work out which direction the correction should go
        if temp < optimal:
            correction = 'increase'
        else:
            correction = 'decrease'

        data = {
            'plant_id': plant_id,
            'correction': correction,
            'current_temp': temp,
            'optimal_temp': optimal
        }

        msg = String()
        msg.data = json.dumps(data)
        self.correction_pub.publish(msg)

        self.get_logger().info(f'Correction sent: {correction} temp for {plant_id}')

def main(args=None):
    rclpy.init(args=args)
    node = PlantMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    