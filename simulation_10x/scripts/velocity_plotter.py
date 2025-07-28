#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np
import threading
import queue
import time
import signal
import sys

class VelocityPlotter(Node):
    def __init__(self):
        super().__init__('velocity_plotter')
        
        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Subscriber for movement status
        self.status_subscriber = self.create_subscription(
            String,
            '/movement_status',
            self.status_callback,
            10
        )
        
        # Data storage - use lists instead of deque to preserve all data
        self.time_data = []
        self.velocity_data = {}
        self.joint_names = []
        self.start_time = None
        self.recording = False  # Flag to control when to start recording
        
        # Display window settings
        self.display_window = 30.0  # Show last 30 seconds in the plot
        
        # Thread-safe queue for data transfer
        self.data_queue = queue.Queue()
        
        # Animation reference
        self.ani = None
        
        # Shutdown flag
        self.shutdown_requested = False
        
        # Setup the plot
        self.setup_plot()
        
        self.get_logger().info("Velocity plotter node started. Waiting for start signal...")
    
    def status_callback(self, msg):
        """Callback for movement status messages"""
        if msg.data == 'start':
            self.get_logger().info("Start signal received. Beginning data recording.")
            self.recording = True
            self.start_time = time.time()
        elif msg.data == 'complete':
            self.get_logger().info("Movement complete signal received. Saving plot and shutting down.")
            self.recording = False
            self.save_plot()
            self.shutdown_requested = True
            # Stop the animation and close the plot window
            if self.ani:
                self.ani.event_source.stop()
            plt.close(self.fig)

    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        # Only record data when recording is enabled
        if not self.recording:
            return
            
        if self.start_time is None:
            self.start_time = time.time()
            
        if not self.joint_names:
            self.joint_names = msg.name
            # Initialize velocity data storage for each joint
            for joint_name in self.joint_names:
                self.velocity_data[joint_name] = []
        
        current_time = time.time() - self.start_time
        
        # Store data in thread-safe queue
        data_point = {
            'time': current_time,
            'joint_names': msg.name,
            'velocities': msg.velocity
        }
        
        try:
            self.data_queue.put_nowait(data_point)
        except queue.Full:
            # If queue is full, remove oldest item and add new one
            try:
                self.data_queue.get_nowait()
                self.data_queue.put_nowait(data_point)
            except queue.Empty:
                pass
    
    def setup_plot(self):
        """Setup the matplotlib plot"""
        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.ax.set_title('Live Joint Velocities vs Time', fontsize=16, fontweight='bold')
        self.ax.set_xlabel('Time (seconds)', fontsize=12)
        self.ax.set_ylabel('Velocity (rad/s)', fontsize=12)
        self.ax.grid(True, alpha=0.3)
        
        # Colors for different joints
        self.colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4', '#FFEAA7', '#DDA0DD']
        self.lines = {}
        
        # Set up the plot limits
        self.ax.set_xlim(0, self.display_window)
        self.ax.set_ylim(-3, 3)  # Typical joint velocity range
        
        # Add legend placeholder
        self.ax.legend(loc='upper right')
        
        plt.tight_layout()
    
    def update_plot_data(self):
        """Update plot data from the queue"""
        # Process all available data from the queue
        while not self.data_queue.empty():
            try:
                data_point = self.data_queue.get_nowait()
                
                # Add time point
                self.time_data.append(data_point['time'])
                
                # Add velocity data for each joint
                for i, joint_name in enumerate(data_point['joint_names']):
                    if joint_name in self.velocity_data:
                        if i < len(data_point['velocities']):
                            velocity = data_point['velocities'][i]
                            self.velocity_data[joint_name].append(velocity)
                        else:
                            self.velocity_data[joint_name].append(0.0)
                
            except queue.Empty:
                break
    
    def get_display_data(self):
        """Get data within the display window"""
        if len(self.time_data) == 0:
            return [], {}
        
        current_time = self.time_data[-1]
        start_time = current_time - self.display_window
        
        # Find the start index for the display window
        start_idx = 0
        for i, t in enumerate(self.time_data):
            if t >= start_time:
                start_idx = i
                break
        
        # Get time data for display
        display_time = self.time_data[start_idx:]
        
        # Get velocity data for display
        display_velocities = {}
        for joint_name, velocities in self.velocity_data.items():
            if len(velocities) > start_idx:
                display_velocities[joint_name] = velocities[start_idx:]
            else:
                display_velocities[joint_name] = []
        
        return display_time, display_velocities
    
    def animate(self, frame):
        """Animation function for matplotlib"""
        self.update_plot_data()
        
        if len(self.time_data) < 2:
            return list(self.lines.values())
        
        # Get data for the current display window
        display_time, display_velocities = self.get_display_data()
        
        if len(display_time) == 0:
            return list(self.lines.values())
        
        # Convert to numpy arrays for plotting
        time_array = np.array(display_time)
        
        # Update or create lines for each joint
        legend_labels = []
        lines_to_return = []
        
        for i, (joint_name, velocities) in enumerate(display_velocities.items()):
            if len(velocities) > 0:
                velocity_array = np.array(velocities)
                color = self.colors[i % len(self.colors)]
                
                # Only plot if we have matching time and velocity data
                if len(velocity_array) == len(time_array):
                    if joint_name not in self.lines:
                        # Create new line
                        line, = self.ax.plot(time_array, velocity_array, 
                                           color=color, linewidth=2, alpha=0.8, label=joint_name)
                        self.lines[joint_name] = line
                    else:
                        # Update existing line with new data
                        self.lines[joint_name].set_data(time_array, velocity_array)
                    
                    legend_labels.append(joint_name)
                    lines_to_return.append(self.lines[joint_name])
        
        # Update plot limits dynamically
        if len(time_array) > 0:
            # Show rolling window
            max_time = time_array[-1]
            self.ax.set_xlim(max_time - self.display_window, max_time + 1)
            
            # Auto-scale y-axis based on current visible data
            all_velocities = []
            for velocities in display_velocities.values():
                if len(velocities) > 0:
                    all_velocities.extend(velocities)
            
            if all_velocities:
                y_min = min(all_velocities)
                y_max = max(all_velocities)
                y_margin = max(0.1, (y_max - y_min) * 0.1)
                self.ax.set_ylim(y_min - y_margin, y_max + y_margin)
        
        # Update legend only if it changed
        if legend_labels and (not hasattr(self, '_last_legend_labels') or 
                             self._last_legend_labels != legend_labels):
            self.ax.legend(legend_labels, loc='upper right', fontsize=10)
            self._last_legend_labels = legend_labels.copy()
        
        # Update statistics text
        if len(time_array) > 0:
            total_points = len(self.time_data)
            visible_points = len(time_array)
            stats_text = f"Time: {time_array[-1]:.1f}s | Total Points: {total_points} | Visible: {visible_points}"
            if hasattr(self, '_stats_text'):
                self._stats_text.remove()
            self._stats_text = self.ax.text(0.02, 0.98, stats_text, transform=self.ax.transAxes,
                                          fontsize=10, verticalalignment='top',
                                          bbox=dict(boxstyle='round', facecolor='black', alpha=0.5))
        
        return lines_to_return
    
    def save_plot(self):
        """Save the final plot to a file"""
        try:
            self.update_plot_data() # Ensure all data is processed
            
            # Only save if we have data
            if not self.time_data:
                self.get_logger().warn("No data to save")
                return
            
            # Set final plot limits to show all data
            self.ax.set_xlim(0, self.time_data[-1] + 1)
                
            all_velocities = []
            for velocities in self.velocity_data.values():
                if velocities:
                    all_velocities.extend(velocities)
            
            if all_velocities:
                y_min = min(all_velocities)
                y_max = max(all_velocities)
                y_margin = max(0.1, (y_max - y_min) * 0.1)
                self.ax.set_ylim(y_min - y_margin, y_max + y_margin)

            # Redraw all lines with all data points
            for joint_name, line in self.lines.items():
                if joint_name in self.velocity_data and len(self.velocity_data[joint_name]) == len(self.time_data):
                    line.set_data(self.time_data, self.velocity_data[joint_name])

            # Save with timestamp to avoid overwriting
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f'velocity_plot_{timestamp}.png'
            self.fig.savefig(filename, dpi=300, bbox_inches='tight')
            self.get_logger().info(f"Plot saved to {filename}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save plot: {e}")

    def run_plot(self):
        """Run the matplotlib animation"""
        # Setup signal handlers for graceful shutdown
        def signal_handler(signum, frame):
            self.get_logger().info(f"Received signal {signum}, shutting down gracefully...")
            self.shutdown_requested = True
            if self.ani:
                self.ani.event_source.stop()
            plt.close(self.fig)
        
        signal.signal(signal.SIGTERM, signal_handler)
        signal.signal(signal.SIGINT, signal_handler)
        
        # Start the animation
        self.ani = animation.FuncAnimation(
            self.fig, self.animate, interval=50, blit=False, cache_frame_data=False
        )

        try:
            # Show the plot - this will block until the window is closed
            plt.show()
        except Exception as e:
            self.get_logger().error(f"Error in plot display: {e}")
        finally:
            # Ensure cleanup happens
            if not self.shutdown_requested:
                self.save_plot()

def main():
    # Initialize ROS2
    rclpy.init()
    velocity_plotter = None
    
    try:
        # Create the node
        velocity_plotter = VelocityPlotter()

        # Start ROS2 spinning in a separate thread
        def ros_spin():
            try:
                while rclpy.ok() and not velocity_plotter.shutdown_requested:
                    rclpy.spin_once(velocity_plotter, timeout_sec=0.1)
            except Exception as e:
                velocity_plotter.get_logger().error(f"Error in ROS spinning: {e}")

        ros_thread = threading.Thread(target=ros_spin, daemon=True)
        ros_thread.start()

        # Run the plot in the main thread
        velocity_plotter.run_plot()

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("\nShutting down velocity plotter...")
    except Exception as e:
        print(f"Error in velocity plotter: {e}")
    finally:
        # Cleanup
        if velocity_plotter is not None:
            velocity_plotter.shutdown_requested = True
            # Save plot on shutdown if we have data
            if velocity_plotter.time_data:
                velocity_plotter.save_plot()
            if rclpy.ok():
                velocity_plotter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 