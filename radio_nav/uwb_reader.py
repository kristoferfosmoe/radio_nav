#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32

# This file reads a UWB radio and publishes the distance and ID to the other beacon

class UWBReader(Node):

    def __init__(self):
        super().__init__('uwb_reader')

        # Declare parameters with default values
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        # Get parameters
        port = self.get_parameter('port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(port, baud_rate, timeout=1.0)
            self.get_logger().info(f'Connected to UWB on {port} at {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to GPS: {str(e)}')
            raise

        # Create publishers
        self.distance_pub = self.create_publisher(Float32, 'uwb/distance', 10)
        
        # Create timer for reading GPS data
        self.create_timer(0.1, self.read_uwb)

    def read_uwb(self):
        try:
            if self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('DIST:'):
                    distance = float(line.split(' ')[1])
                    self.publish_distance(distance)                
        except Exception as e:
            self.get_logger().error(f'Error reading UWB: {str(e)}')

    def publish_distance(self, distance):
        # Create Float32 message
        distance_msg = Float32()
        distance_msg.data = distance

        # Publish the distance
        self.distance_pub.publish(distance_msg)
        self.get_logger().info(f'Published UWB distance: {distance}')

def main(args=None):
    rclpy.init(args=args)
    node = UWBReader()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()