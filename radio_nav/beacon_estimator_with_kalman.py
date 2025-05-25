#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
import math
from scipy.optimize import least_squares
from typing import List, Tuple

class BeaconEstimatorWithKalman(Node):

    def __init__(self):
        super().__init__('beacon_estimator_with_kalman')
        
        # Store the latest GPS location and UWB distance
        self.current_gps = None
        self.current_distance = None
        self.positions = []  # List to store GPS positions
        self.ranges = []     # List to store UWB distances
        
        # Create subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix,
            'gps/filtered_location',  # Subscribe to filtered GPS data
            self.gps_callback,
            10)
            
        self.uwb_sub = self.create_subscription(
            Float32,
            'uwb/distance',
            self.uwb_callback,
            10)
            
        # Create publisher for estimated beacon location
        self.beacon_pub = self.create_publisher(
            PointStamped,
            'beacon/estimated_location',
            10)
            
        # Create timer for estimation
        self.create_timer(.5, self.estimate_beacon_location)
        
    def gps_callback(self, msg):
        self.current_gps = msg
        self.get_logger().debug(f'Received filtered GPS: {msg.latitude}, {msg.longitude}')
        # Append GPS location to positions list
        if self.current_distance is not None:  # Only store positions when we have a corresponding range
            self.positions.append((msg.latitude, msg.longitude))
        
    def uwb_callback(self, msg):
        self.current_distance = msg.data
        self.get_logger().debug(f'Received UWB distance: {self.current_distance}')
        # Append UWB distance to ranges list
        if self.current_gps is not None:  # Only store ranges when we have a corresponding position
            distance_in_degrees = self.meters_to_decimal_degree(msg.data)
            self.ranges.append(distance_in_degrees)
        
    def meters_to_decimal_degree(self, meters):
        """
        Convert meters to a single decimal degree value.
        
        Args:
            meters: Distance in meters
            
        Returns:
            Single decimal degree value
        """
        # Earth's radius in meters
        earth_radius = 6378137.0
        
        # Convert meters to decimal degrees (1 degree is approximately 111,320 meters)
        decimal_degree = (meters / earth_radius) * (180.0 / math.pi)
        
        return decimal_degree
    
    def _multilaterate(self, positions: List[Tuple[float, float]], ranges: List[float]) -> Tuple[float, float]:
        """
        Perform multilateration using least squares optimization.
        
        Args:
            positions: List of agent positions (latitude, longitude)
            ranges: List of range measurements (UWB distances)
            
        Returns:
            Estimated position of the target beacon (latitude, longitude)
        """
        # Define the objective function for least squares
        def residuals(pos):
            x, y = pos
            res = []
            for i, agent_pos in enumerate(positions):
                # Calculate the distance between the estimated position and the agent
                calculated_range = math.sqrt((x - agent_pos[0])**2 + (y - agent_pos[1])**2)
                # The residual is the difference between the calculated and measured range
                res.append(calculated_range - ranges[i])
            return res
        
        # Initial guess is our current location
        initial_guess = [self.current_gps.latitude, self.current_gps.longitude]
        
        # Perform least squares optimization
        result = least_squares(residuals, initial_guess)
        
        # Extract the optimized position
        x_opt, y_opt = result.x
        
        return (x_opt, y_opt)
        
    def estimate_beacon_location(self):
        if self.current_gps is None or self.current_distance is None:
            self.get_logger().info('Waiting for GPS and UWB data...')
        
        else:
            # Get current GPS and UWB data
            x, y = self.current_gps.latitude, self.current_gps.longitude
            distance = self.current_distance
            
            # Convert UWB distance to decimal degrees
            distance_in_degrees = self.meters_to_decimal_degree(distance)
            
            # If we have enough data points, perform multilateration
            if len(self.positions) >= 3 and len(self.ranges) >= 3:
                # Ensure we have the same number of positions and ranges
                min_length = min(len(self.positions), len(self.ranges))
                positions = self.positions[:min_length]
                ranges = self.ranges[:min_length]
                
                # Perform multilateration
                self.estimated_position = self._multilaterate(positions, ranges)
                
                # Create and publish the estimated location
                beacon_location = PointStamped()
                beacon_location.header.stamp = self.get_clock().now().to_msg()
                beacon_location.header.frame_id = "map"
                beacon_location.point.x = self.estimated_position[0]
                beacon_location.point.y = self.estimated_position[1]
                beacon_location.point.z = 0.0
                
                # Publish the estimated location
                self.beacon_pub.publish(beacon_location)
                self.get_logger().info(f'Published estimated target beacon location: ({self.estimated_position[0]}, {self.estimated_position[1]})')
           
def main(args=None):
    rclpy.init(args=args)
    node = BeaconEstimatorWithKalman()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()