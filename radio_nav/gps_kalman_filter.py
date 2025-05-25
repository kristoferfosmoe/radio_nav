#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import numpy as np
from typing import Optional

class GPSKalmanFilter(Node):
    """
    A ROS2 node that applies a Kalman filter to GPS readings to reduce noise
    and improve position accuracy.
    """

    def __init__(self):
        super().__init__('gps_kalman_filter')
        
        # Declare parameters with default values
        self.declare_parameter('process_noise_lat', 1e-5)  # Process noise for latitude
        self.declare_parameter('process_noise_lon', 1e-5)  # Process noise for longitude
        self.declare_parameter('measurement_noise_lat', 1e-3)  # Measurement noise for latitude
        self.declare_parameter('measurement_noise_lon', 1e-3)  # Measurement noise for longitude
        
        # Get parameters
        self.process_noise_lat = self.get_parameter('process_noise_lat').get_parameter_value().double_value
        self.process_noise_lon = self.get_parameter('process_noise_lon').get_parameter_value().double_value
        self.measurement_noise_lat = self.get_parameter('measurement_noise_lat').get_parameter_value().double_value
        self.measurement_noise_lon = self.get_parameter('measurement_noise_lon').get_parameter_value().double_value
        
        # Initialize Kalman filter state
        self.x = np.zeros(4)  # State vector [lat, lon, lat_vel, lon_vel]
        self.P = np.eye(4)    # State covariance matrix
        
        # Process noise covariance matrix Q
        self.Q = np.diag([
            self.process_noise_lat,
            self.process_noise_lon,
            self.process_noise_lat * 10,
            self.process_noise_lon * 10
        ])
        
        # Measurement noise covariance matrix R
        self.R = np.diag([self.measurement_noise_lat, self.measurement_noise_lon])
        
        # Measurement matrix H (maps state to measurements)
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],  # Latitude measurement
            [0.0, 1.0, 0.0, 0.0]   # Longitude measurement
        ])
        
        # State transition matrix F (for constant velocity model)
        self.F = None  # Will be updated with dt in predict step
        
        # Time of last update
        self.last_time: Optional[float] = None
        
        # Create subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix,
            'gps/location',
            self.gps_callback,
            10)
            
        # Create publishers
        self.filtered_pub = self.create_publisher(
            NavSatFix,
            'gps/filtered_location',
            10)
        
        self.get_logger().info('GPS Kalman Filter initialized')
    
    def predict(self, dt):
        """
        Predict step of the Kalman filter.
        
        Args:
            dt: Time delta since last update in seconds
        """
        # Update state transition matrix with current dt
        self.F = np.array([
            [1.0, 0.0, dt, 0.0],   # lat = lat + dt * lat_vel
            [0.0, 1.0, 0.0, dt],   # lon = lon + dt * lon_vel
            [0.0, 0.0, 1.0, 0.0],  # lat_vel = lat_vel
            [0.0, 0.0, 0.0, 1.0]   # lon_vel = lon_vel
        ])
        
        # Predict state
        self.x = self.F @ self.x
        
        # Predict covariance
        self.P = self.F @ self.P @ self.F.T + self.Q
    
    def update(self, z):
        """
        Update step of the Kalman filter.
        
        Args:
            z: Measurement vector [lat, lon]
        """
        # Calculate innovation (measurement residual)
        y = z - self.H @ self.x
        
        # Calculate innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Calculate Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        
        # Update covariance
        I = np.eye(self.x.shape[0])
        self.P = (I - K @ self.H) @ self.P
    
    def gps_callback(self, msg):
        """
        Callback for GPS messages. Applies Kalman filter and publishes filtered position.
        
        Args:
            msg: NavSatFix message containing raw GPS data
        """
        # Get current time
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Initialize filter with first measurement if needed
        if self.last_time is None:
            self.x[0] = msg.latitude
            self.x[1] = msg.longitude
            self.last_time = current_time
            self.get_logger().info(f'Initialized Kalman filter with position: {msg.latitude}, {msg.longitude}')
            return
        
        # Calculate time delta
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Skip if dt is too small (avoid numerical issues)
        if dt < 1e-6:
            return
            
        # Predict step
        self.predict(dt)
        
        # Update step with measurement
        z = np.array([msg.latitude, msg.longitude])
        self.update(z)
        
        # Create filtered NavSatFix message
        filtered_msg = NavSatFix()
        filtered_msg.header = msg.header
        filtered_msg.latitude = self.x[0]
        filtered_msg.longitude = self.x[1]
        filtered_msg.altitude = msg.altitude  # Pass through altitude unchanged
        filtered_msg.position_covariance_type = msg.position_covariance_type
        
        # Set position covariance from Kalman filter's covariance matrix
        # Only set the lat/lon diagonal elements
        if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            filtered_msg.position_covariance = list(msg.position_covariance)
            filtered_msg.position_covariance[0] = self.P[0, 0]  # Latitude variance
            filtered_msg.position_covariance[4] = self.P[1, 1]  # Longitude variance
        
        # Publish filtered position
        self.filtered_pub.publish(filtered_msg)
        self.get_logger().debug(f'Published filtered GPS: {filtered_msg.latitude}, {filtered_msg.longitude}')


def main(args=None):
    rclpy.init(args=args)
    node = GPSKalmanFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()