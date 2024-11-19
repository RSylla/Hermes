#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import yaml
import tkinter as tk
from tkinter import messagebox, ttk
import datetime
import math
import os
import webbrowser
import folium
from folium import plugins
import threading
import http.server
import socketserver
from pyproj import Transformer
from functools import partial
import json

class GPSWaypointLogger(Node):
    def __init__(self):
        super().__init__('gps_waypoint_logger')
        
        # Initialize the GUI
        self.window = tk.Tk()
        self.window.title("Enhanced GPS Waypoint Logger")
        self.window.geometry("800x600")
        
        # Initialize map server
        self.map_port = 8000
        self.start_map_server()
        
        # Initialize transformer
        self.transformer = Transformer.from_crs("EPSG:4326", "EPSG:32633", always_xy=True)
        
        # Create GUI elements
        self.create_gui_elements()
        
        # Initialize waypoints list
        self.waypoints = []
        self.recording_mode = False  # For continuous recording mode
        
        # Subscribe to GPS topic
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )
        
        # Subscribe to odometry for heading
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Initialize current position
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_heading = 0.0
        
        # Create map with initial position
        self.create_map()
        
        # Update GUI periodically
        self.window.after(100, self.update_gui)

    def create_gui_elements(self):
        # Create main frame
        self.main_frame = ttk.Frame(self.window, padding="10")
        self.main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Create left frame for GPS info and controls
        left_frame = ttk.Frame(self.main_frame)
        left_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Current position display
        ttk.Label(left_frame, text="Current Position:").grid(row=0, column=0, sticky=tk.W)
        self.position_label = ttk.Label(left_frame, text="Waiting for GPS...")
        self.position_label.grid(row=1, column=0, columnspan=2, sticky=tk.W)
        
        # Heading display
        ttk.Label(left_frame, text="Current Heading:").grid(row=2, column=0, sticky=tk.W)
        self.heading_label = ttk.Label(left_frame, text="Waiting for data...")
        self.heading_label.grid(row=3, column=0, columnspan=2, sticky=tk.W)
        
        # Recording mode switch
        self.record_mode_var = tk.BooleanVar()
        ttk.Checkbutton(left_frame, text="Continuous Recording", 
                       variable=self.record_mode_var,
                       command=self.toggle_recording).grid(row=4, column=0, sticky=tk.W)
        
        # Record interval entry
        ttk.Label(left_frame, text="Record Interval (s):").grid(row=5, column=0, sticky=tk.W)
        self.record_interval = ttk.Entry(left_frame, width=10)
        self.record_interval.insert(0, "5")
        self.record_interval.grid(row=5, column=1, sticky=tk.W)
        
        # Waypoints list
        ttk.Label(left_frame, text="Recorded Waypoints:").grid(row=6, column=0, sticky=tk.W)
        self.waypoint_listbox = tk.Listbox(left_frame, height=15, width=50)
        self.waypoint_listbox.grid(row=7, column=0, columnspan=2, sticky=(tk.W, tk.E))
        
        # Control buttons
        button_frame = ttk.Frame(left_frame)
        button_frame.grid(row=8, column=0, columnspan=2, pady=10)
        
        ttk.Button(button_frame, text="Record Waypoint", 
                  command=self.record_waypoint).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Save Route", 
                  command=self.save_waypoints).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Clear All", 
                  command=self.clear_waypoints).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Open Map", 
                  command=self.open_map_browser).pack(side=tk.LEFT, padx=5)

    def start_map_server(self):
        """Start a simple HTTP server for the map"""
        handler = partial(http.server.SimpleHTTPRequestHandler, directory=os.getcwd())
        self.httpd = socketserver.TCPServer(("", self.map_port), handler)
        self.server_thread = threading.Thread(target=self.httpd.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()

    def create_map(self):
        """Create an interactive map"""
        # Create map centered at current position or default location
        m = folium.Map(location=[self.current_lat or 0, self.current_lon or 0],
                      zoom_start=15)
        
        # Add click handler for adding waypoints
        m.add_child(folium.ClickForLatLng())
        
        # Add current position marker
        if self.current_lat != 0 and self.current_lon != 0:
            folium.Marker(
                [self.current_lat, self.current_lon],
                popup="Current Position",
                icon=folium.Icon(color='red', icon='info-sign')
            ).add_to(m)
        
        # Add existing waypoints
        for i, wp in enumerate(self.waypoints):
            folium.Marker(
                [wp['latitude'], wp['longitude']],
                popup=f"Waypoint {i+1}",
                icon=folium.Icon(color='blue')
            ).add_to(m)
        
        # Add path if there are waypoints
        if len(self.waypoints) > 1:
            points = [[wp['latitude'], wp['longitude']] for wp in self.waypoints]
            folium.PolyLine(points, weight=2, color='blue', opacity=0.8).add_to(m)
        
        # Save map
        m.save('waypoint_map.html')

    def open_map_browser(self):
        """Open the map in a web browser"""
        self.create_map()  # Update map
        webbrowser.open(f'http://localhost:{self.map_port}/waypoint_map.html')

    def toggle_recording(self):
        """Toggle continuous recording mode"""
        self.recording_mode = self.record_mode_var.get()
        if self.recording_mode:
            interval = float(self.record_interval.get())
            self.create_timer(interval, self.record_waypoint)
        self.get_logger().info(f'Recording mode: {"ON" if self.recording_mode else "OFF"}')

    def gps_callback(self, msg):
        """Handle GPS updates"""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        
        # Convert to UTM for distance calculations
        try:
            if self.recording_mode and self.waypoints:
                x, y = self.transformer.transform(msg.longitude, msg.latitude)
                last_wp = self.waypoints[-1]
                last_x, last_y = self.transformer.transform(last_wp['longitude'], last_wp['latitude'])
                
                # Calculate distance from last waypoint
                dist = math.sqrt((x - last_x)**2 + (y - last_y)**2)
                
                # Record new waypoint if distance threshold is met (e.g., 5 meters)
                if dist > 5.0:
                    self.record_waypoint()
        except Exception as e:
            self.get_logger().error(f'GPS transform error: {str(e)}')

    def odom_callback(self, msg):
        """Handle odometry updates"""
        quaternion = msg.pose.pose.orientation
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        self.current_heading = math.atan2(siny_cosp, cosy_cosp)

    def update_gui(self):
        """Update GUI elements"""
        self.position_label.config(
            text=f"Lat: {self.current_lat:.6f}, Lon: {self.current_lon:.6f}")
        self.heading_label.config(
            text=f"Heading: {math.degrees(self.current_heading):.2f}°")
        self.window.after(100, self.update_gui)

    def record_waypoint(self):
        """Record current position as waypoint"""
        waypoint = {
            'latitude': self.current_lat,
            'longitude': self.current_lon,
            'heading': self.current_heading,
            'utm_x': None,
            'utm_y': None
        }
        
        # Convert to UTM
        try:
            x, y = self.transformer.transform(self.current_lon, self.current_lat)
            waypoint['utm_x'] = x
            waypoint['utm_y'] = y
        except Exception as e:
            self.get_logger().warning(f'UTM conversion failed: {str(e)}')
        
        self.waypoints.append(waypoint)
        self.waypoint_listbox.insert(tk.END, 
            f"Lat: {self.current_lat:.6f}, Lon: {self.current_lon:.6f}, " +
            f"Heading: {math.degrees(self.current_heading):.2f}°")
        
        # Update map
        self.create_map()
        self.get_logger().info(f'Recorded waypoint: {waypoint}')

    def save_waypoints(self):
        """Save waypoints to YAML file"""
        if not self.waypoints:
            messagebox.showwarning("Warning", "No waypoints to save!")
            return
        
        try:
            # Generate filename with timestamp
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            file_path = os.path.join(os.getcwd(), 'waypoints', f'route_{timestamp}.yaml')
            
            # Ensure directory exists
            os.makedirs(os.path.dirname(file_path), exist_ok=True)
            
            # Save waypoints
            with open(file_path, 'w') as f:
                yaml.dump({
                    'waypoints': self.waypoints,
                    'metadata': {
                        'timestamp': timestamp,
                        'num_points': len(self.waypoints),
                        'recording_mode': self.recording_mode
                    }
                }, f)
            
            # Also save as GeoJSON for easy visualization
            geojson_path = os.path.join(os.getcwd(), 'waypoints', f'route_{timestamp}.geojson')
            self.save_as_geojson(geojson_path)
            
            messagebox.showinfo("Success", f"Waypoints saved to {file_path}")
            self.get_logger().info(f'Saved waypoints to {file_path}')
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save waypoints: {str(e)}")
            self.get_logger().error(f'Failed to save waypoints: {str(e)}')

    def save_as_geojson(self, filepath):
        """Save waypoints as GeoJSON file"""
        features = []
        
        # Add points
        for i, wp in enumerate(self.waypoints):
            features.append({
                'type': 'Feature',
                'geometry': {
                    'type': 'Point',
                    'coordinates': [wp['longitude'], wp['latitude']]
                },
                'properties': {
                    'id': i,
                    'heading': wp['heading'],
                    'utm_x': wp['utm_x'],
                    'utm_y': wp['utm_y']
                }
            })
        
        # Add path line
        if len(self.waypoints) > 1:
            features.append({
                'type': 'Feature',
                'geometry': {
                    'type': 'LineString',
                    'coordinates': [[wp['longitude'], wp['latitude']] for wp in self.waypoints]
                },
                'properties': {'type': 'path'}
            })
        
        geojson = {
            'type': 'FeatureCollection',
            'features': features
        }
        
        with open(filepath, 'w') as f:
            json.dump(geojson, f)

    def clear_waypoints(self):
        """Clear all recorded waypoints"""
        self.waypoints.clear()
        self.waypoint_listbox.delete(0, tk.END)
        self.create_map()
        self.get_logger().info('Cleared all waypoints')

def main(args=None):
    rclpy.init(args=args)
    logger_node = GPSWaypointLogger()
    
    try:
        # Run the GUI and ROS node together
        while rclpy.ok():
            rclpy.spin_once(logger_node, timeout_sec=0.1)
            logger_node.window.update()
    except Exception as e:
        logger_node.get_logger().error(f'Error in main loop: {str(e)}')
    finally:
        logger_node.httpd.shutdown()
        logger_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()