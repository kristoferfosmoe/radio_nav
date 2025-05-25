#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
import tkinter as tk
import math
from threading import Thread

class DisplayNav(Node):
    def __init__(self):
        super().__init__('display_nav')
        
        # Store the latest positions
        self.current_position = None
        self.beacon_position = None
        
        # Create subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix,
            'gps/location',
            self.gps_callback,
            10)
            
        self.beacon_sub = self.create_subscription(
            PointStamped,
            'beacon/estimated_location',
            self.beacon_callback,
            10)
        
        # Set up GUI in a separate thread
        self.gui_thread = Thread(target=self.setup_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()
        
        # Create timer for updating the display
        self.create_timer(0.1, self.update_display)
        
        self.get_logger().info('Display Navigation node initialized')
    
    def gps_callback(self, msg):
        self.current_position = (msg.latitude, msg.longitude)
        self.get_logger().debug(f'Received current position: {self.current_position}')
    
    def beacon_callback(self, msg):
        self.beacon_position = (msg.point.x, msg.point.y)
        self.get_logger().debug(f'Received beacon position: {self.beacon_position}')
    
    def setup_gui(self):
        # Create the main window
        self.root = tk.Tk()
        self.root.title("Radio Navigation Display")
        self.root.geometry("750x750")
        
        # Create a canvas for drawing
        self.canvas = tk.Canvas(self.root, bg="white", width=600, height=600)
        self.canvas.pack(pady=20)
        
        # Create labels for position information
        self.info_frame = tk.Frame(self.root)
        self.info_frame.pack(fill=tk.X, padx=10)
        
        self.current_pos_label = tk.Label(self.info_frame, text="Current Position: Waiting for data...")
        self.current_pos_label.pack(anchor=tk.W)
        
        self.beacon_pos_label = tk.Label(self.info_frame, text="Beacon Position: Waiting for data...")
        self.beacon_pos_label.pack(anchor=tk.W)
        
        self.distance_label = tk.Label(self.info_frame, text="Distance: Waiting for data...")
        self.distance_label.pack(anchor=tk.W)
        
        self.bearing_label = tk.Label(self.info_frame, text="Bearing: Waiting for data...")
        self.bearing_label.pack(anchor=tk.W)
        
        # Start the GUI main loop
        self.root.mainloop()
    
    def update_display(self):
        if not hasattr(self, 'root') or not self.root.winfo_exists():
            return
            
        if self.current_position and self.beacon_position:
            # Update position labels
            self.current_pos_label.config(text=f"Current Position: {self.current_position[0]:.6f}, {self.current_position[1]:.6f}")
            self.beacon_pos_label.config(text=f"Beacon Position: {self.beacon_position[0]:.6f}, {self.beacon_position[1]:.6f}")
            
            # Calculate distance and bearing
            distance, bearing = self.calculate_distance_bearing(
                self.current_position[0], self.current_position[1],
                self.beacon_position[0], self.beacon_position[1]
            )
            
            self.distance_label.config(text=f"Distance: {distance:.2f} meters")
            self.bearing_label.config(text=f"Bearing: {bearing:.1f}°")
            
            # Clear canvas
            self.canvas.delete("all")
            
            # Draw the map (simple representation)
            canvas_width = 600
            canvas_height = 600
            center_x = canvas_width // 2
            center_y = canvas_height // 2
            
            # Draw current position (blue dot)
            self.canvas.create_oval(center_x-5, center_y-5, center_x+5, center_y+5, fill="blue", tags="current")
            self.canvas.create_text(center_x, center_y+20, text="You", fill="blue")
            
            # Calculate beacon position on canvas
            # For simplicity, we'll use a fixed scale and place the beacon relative to current position
            scale_factor = 10000000  # pixels per degree difference
            dx = (self.beacon_position[1] - self.current_position[1]) * scale_factor
            dy = (self.beacon_position[0] - self.current_position[0]) * -scale_factor  # Negative because y increases downward in canvas
            
            beacon_x = center_x + dx
            beacon_y = center_y + dy

            # Draw beacon position (red dot)
            self.canvas.create_oval(beacon_x-5, beacon_y-5, beacon_x+5, beacon_y+5, fill="red", tags="beacon")
            self.canvas.create_text(beacon_x, beacon_y+20, text="Beacon", fill="red")
            
            # Draw arrow from current position to beacon
            arrow_length = min(math.sqrt(dx*dx + dy*dy) - 20, 100)  # Limit arrow length
            if arrow_length > 10:  # Only draw if there's enough distance
                arrow_dx = dx * arrow_length / math.sqrt(dx*dx + dy*dy)
                arrow_dy = dy * arrow_length / math.sqrt(dx*dx + dy*dy)
                
                # Draw arrow line
                self.canvas.create_line(center_x, center_y, center_x + arrow_dx, center_y + arrow_dy, 
                                       arrow=tk.LAST, width=3, fill="green", tags="arrow")
    
    def calculate_distance_bearing(self, lat1, lon1, lat2, lon2):
        """
        Calculate distance (in meters) and bearing (in degrees) between two lat/lon points
        """
        # Earth radius in meters
        R = 6371000
        
        # Convert to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Differences
        dlon = lon2_rad - lon1_rad
        dlat = lat2_rad - lat1_rad
        
        # Haversine formula for distance
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = R * c
        
        # Calculate bearing
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        bearing = math.degrees(math.atan2(y, x))
        
        # Normalize bearing to 0-360
        bearing = (bearing + 360) % 360
        
        return distance, bearing

def main(args=None):
    rclpy.init(args=args)
    node = DisplayNav()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()