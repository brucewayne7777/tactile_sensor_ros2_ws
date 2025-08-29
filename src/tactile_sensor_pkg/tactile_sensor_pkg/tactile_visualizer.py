#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool
import os

# Set matplotlib backend BEFORE importing matplotlib
import matplotlib
# Check environment and set appropriate backend
if 'DISPLAY' in os.environ:
    try:
        matplotlib.use('TkAgg')
    except:
        try:
            matplotlib.use('Qt5Agg')
        except:
            matplotlib.use('Agg')
else:
    matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import time

class TactileVisualizer(Node):
    def __init__(self):
        super().__init__('tactile_visualizer')
        
        # Data storage for plotting
        self.raw_data_history = deque(maxlen=100)  # Store last 100 readings
        self.binary_data_history = deque(maxlen=100)
        self.touch_history = deque(maxlen=100)
        self.timestamps = deque(maxlen=100)
        
        # Initialize data
        self.raw_values = [0, 0, 0, 0]
        self.binary_values = [0, 0, 0, 0]
        self.touch_detected = False
        
        # Create subscribers
        self.raw_subscription = self.create_subscription(
            Int32MultiArray,
            'tactile_raw_values',
            self.raw_callback,
            10
        )
        
        self.binary_subscription = self.create_subscription(
            Int32MultiArray,
            'tactile_binary_array',
            self.binary_callback,
            10
        )
        
        self.touch_subscription = self.create_subscription(
            Bool,
            'tactile_touch_detected',
            self.touch_callback,
            10
        )
        
        # Check if we can display plots
        self.can_display = self.check_display()
        
        if self.can_display:
            # Set up plotting
            self.setup_plots()
            # Create a timer for updating plots in the main thread
            self.plot_timer = self.create_timer(0.1, self.update_plots)  # Update every 100ms
            self.get_logger().info('Tactile Visualizer initialized with GUI. Press Ctrl+C to stop.')
        else:
            # Create a timer for text-based updates
            self.text_timer = self.create_timer(0.5, self.update_text_display)  # Update every 500ms
            self.get_logger().info('Tactile Visualizer initialized with text display. Press Ctrl+C to stop.')

    def check_display(self):
        """Check if we can display GUI plots"""
        try:
            # Check if DISPLAY is set and accessible
            if 'DISPLAY' in os.environ:
                # Try to create a simple plot to test
                test_fig, test_ax = plt.subplots(1, 1, figsize=(1, 1))
                plt.close(test_fig)
                return True
            else:
                return False
        except Exception as e:
            self.get_logger().warn(f'GUI display not available: {e}')
            return False

    def raw_callback(self, msg):
        """Callback for raw sensor values"""
        self.raw_values = msg.data
        self.raw_data_history.append(list(msg.data))
        self.timestamps.append(time.time())

    def binary_callback(self, msg):
        """Callback for binary sensor states"""
        self.binary_values = msg.data
        self.binary_data_history.append(list(msg.data))

    def touch_callback(self, msg):
        """Callback for overall touch detection"""
        self.touch_detected = msg.data
        self.touch_history.append(1 if msg.data else 0)

    def setup_plots(self):
        """Set up matplotlib plots"""
        plt.ion()  # Turn on interactive mode
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(12, 10))
        self.fig.suptitle('Tactile Sensor Data Visualization', fontsize=16, fontweight='bold')
        
        # Raw values plot
        self.ax1.set_title('Raw Sensor Values (ADC 0-1023)', fontsize=12, fontweight='bold')
        self.ax1.set_ylabel('Analog Value', fontsize=10)
        self.ax1.set_ylim(0, 1024)
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_facecolor('#f8f9fa')
        
        # Binary values plot
        self.ax2.set_title('Binary Touch States (0/1)', fontsize=12, fontweight='bold')
        self.ax2.set_ylabel('Touch State', fontsize=10)
        self.ax2.set_ylim(-0.1, 1.1)
        self.ax2.grid(True, alpha=0.3)
        self.ax2.set_facecolor('#f8f9fa')
        
        # Overall touch plot
        self.ax3.set_title('Overall Touch Detection', fontsize=12, fontweight='bold')
        self.ax3.set_ylabel('Touch Detected', fontsize=10)
        self.ax3.set_ylim(-0.1, 1.1)
        self.ax3.grid(True, alpha=0.3)
        self.ax3.set_facecolor('#f8f9fa')
        
        # Set colors for sensors
        self.colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4']  # Red, Teal, Blue, Green
        
        plt.tight_layout()
        plt.show(block=False)

    def update_plots(self):
        """Update the plots with current data"""
        if len(self.raw_data_history) == 0:
            return
            
        try:
            # Clear previous plots
            self.ax1.clear()
            self.ax2.clear()
            self.ax3.clear()
            
            # Convert deques to lists for plotting
            raw_data = list(self.raw_data_history)
            binary_data = list(self.binary_data_history)
            touch_data = list(self.touch_history)
            
            # Create time axis
            time_axis = np.arange(len(raw_data))
            
            # Plot raw values
            self.ax1.set_title('Raw Sensor Values (ADC 0-1023)', fontsize=12, fontweight='bold')
            self.ax1.set_ylabel('Analog Value', fontsize=10)
            self.ax1.set_ylim(0, 1024)
            self.ax1.grid(True, alpha=0.3)
            self.ax1.set_facecolor('#f8f9fa')
            
            for i in range(4):
                sensor_data = [data[i] for data in raw_data]
                self.ax1.plot(time_axis, sensor_data, 
                             color=self.colors[i], 
                             linewidth=2, 
                             label=f'Sensor {i+1}',
                             marker='o', 
                             markersize=3)
            self.ax1.legend(loc='upper right')
            
            # Plot binary values
            self.ax2.set_title('Binary Touch States (0/1)', fontsize=12, fontweight='bold')
            self.ax2.set_ylabel('Touch State', fontsize=10)
            self.ax2.set_ylim(-0.1, 1.1)
            self.ax2.grid(True, alpha=0.3)
            self.ax2.set_facecolor('#f8f9fa')
            
            for i in range(4):
                sensor_data = [data[i] for data in binary_data]
                self.ax2.plot(time_axis, sensor_data, 
                             color=self.colors[i], 
                             linewidth=2, 
                             label=f'Sensor {i+1}',
                             marker='s', 
                             markersize=4)
            self.ax2.legend(loc='upper right')
            
            # Plot overall touch
            self.ax3.set_title('Overall Touch Detection', fontsize=12, fontweight='bold')
            self.ax3.set_ylabel('Touch Detected', fontsize=10)
            self.ax3.set_ylim(-0.1, 1.1)
            self.ax3.grid(True, alpha=0.3)
            self.ax3.set_facecolor('#f8f9fa')
            
            # Color the overall touch line based on current state
            touch_color = '#FF6B6B' if self.touch_detected else '#4ECDC4'
            self.ax3.plot(time_axis, touch_data, 
                         color=touch_color, 
                         linewidth=3, 
                         label=f'Touch Detected: {"YES" if self.touch_detected else "NO"}',
                         marker='D', 
                         markersize=5)
            self.ax3.legend(loc='upper right')
            
            # Add current values as text
            if len(raw_data) > 0:
                current_raw = raw_data[-1]
                current_binary = binary_data[-1]
                
                # Add text annotations for current values
                for i, (raw_val, bin_val) in enumerate(zip(current_raw, current_binary)):
                    self.ax1.text(len(raw_data)-1, raw_val, f' {raw_val}', 
                                 fontsize=8, verticalalignment='center')
                    self.ax2.text(len(binary_data)-1, bin_val, f' {bin_val}', 
                                 fontsize=8, verticalalignment='center')
            
            plt.tight_layout()
            plt.draw()
            plt.pause(0.01)
            
        except Exception as e:
            self.get_logger().warn(f'Error updating plots: {e}')

    def update_text_display(self):
        """Update text-based display when GUI is not available"""
        if len(self.raw_data_history) == 0:
            return
            
        # Clear screen
        os.system('clear' if os.name == 'posix' else 'cls')
        
        # Header
        print("=" * 80)
        print("                    TACTILE SENSOR DATA VISUALIZATION")
        print("=" * 80)
        print()
        
        # Current values section
        print("CURRENT SENSOR VALUES:")
        print("-" * 40)
        
        # Raw values with bars
        print("Raw ADC Values (0-1023):")
        for i, val in enumerate(self.raw_values):
            bar = self.create_bar(val)
            touch_status = " [TOUCHED!]" if self.binary_values[i] else ""
            print(f"  Sensor {i+1}: {val:4d} {bar} {touch_status}")
        
        print()
        
        # Binary states
        print("Binary Touch States:")
        for i, val in enumerate(self.binary_values):
            bar = self.create_binary_bar(val)
            status = "TOUCHED" if val else "NOT TOUCHED"
            print(f"  Sensor {i+1}: {bar} {status}")
        
        print()
        
        # Overall touch status
        overall_status = "YES - TOUCH DETECTED!" if self.touch_detected else "NO - NO TOUCH"
        print(f"Overall Touch Detection: {overall_status}")
        
        print()
        print("-" * 40)
        
        # Recent history (last 5 readings)
        if len(self.raw_data_history) > 1:
            print("RECENT HISTORY (last 5 readings):")
            print("-" * 40)
            
            recent_raw = list(self.raw_data_history)[-5:]
            recent_binary = list(self.binary_data_history)[-5:]
            
            print("Raw Values:")
            for i, reading in enumerate(recent_raw):
                print(f"  {i+1}: {reading}")
            
            print("\nBinary States:")
            for i, reading in enumerate(recent_binary):
                binary_str = ''.join(['█' if x else '░' for x in reading])
                print(f"  {i+1}: {binary_str} {reading}")
        
        print()
        print("=" * 80)
        print("Press Ctrl+C to stop")

    def create_bar(self, value, max_value=1023, width=20):
        """Create a text-based bar chart"""
        filled = int((value / max_value) * width)
        bar = '█' * filled + '░' * (width - filled)
        return bar

    def create_binary_bar(self, value, width=10):
        """Create a binary bar (filled or empty)"""
        return '█' * width if value else '░' * width

def main(args=None):
    rclpy.init(args=args)
    visualizer = TactileVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        print("\n\nVisualizer stopped by user.")
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
