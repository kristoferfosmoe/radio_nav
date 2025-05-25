#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import pynmea2

# This file reads a GPS device and publishes the location as a NavSatFix message


class GPSNode(Node):

    def __init__(self):
        super().__init__('gps_node')
        
        # Declare parameters with default values
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        
        # Get parameters
        port = self.get_parameter('port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(port, baud_rate, timeout=1.0)
            self.get_logger().info(f'Connected to GPS on {port} at {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to GPS: {str(e)}')
            raise
        
        # Create publishers
        self.location_pub = self.create_publisher(NavSatFix, 'gps/location', 10)
        
        # Create timer for reading GPS data
        self.create_timer(0.1, self.read_gps)

    def read_gps(self):
        try:
            if self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('$'):
                    try:
                        msg = pynmea2.parse(line)

                        # Parse GLL containing latitude and longitude
                        if isinstance(msg, pynmea2.GLL):
                            self.publish_location(msg)
                            # self.get_logger().info(f'Parsed GLL: {line}')
                    except pynmea2.ParseError as e:
                        self.get_logger().info(f'Failed to parse NMEA: {e}')
        except Exception as e:
            self.get_logger().error(f'Error reading GPS: {str(e)}')
    
    def publish_location(self, msg):
        # Create NavSatFix message
        location = NavSatFix()
        location.header.stamp = self.get_clock().now().to_msg()
        location.header.frame_id = "gps"
        
        # Set coordinates if available
        if msg.latitude and msg.longitude:
            location.latitude = msg.latitude
            location.longitude = msg.longitude
            
            # Publish the location
            self.location_pub.publish(location)
            self.get_logger().info(f'Published GPS location: {msg.latitude}, {msg.longitude}')

    def __del__(self):
        if hasattr(self, 'serial_conn') and self.serial_conn.is_open:
            self.serial_conn.close()


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode() # starts the node
    rclpy.spin(node) # keeps the node alive
    
    # Clean up
    if hasattr(node, 'serial_conn') and node.serial_conn.is_open:
        node.serial_conn.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
