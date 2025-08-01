#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time
import signal
import sys
import os
import threading
from collections import deque

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
        
        # Data storage
        self.time_data = []
        self.velocity_data = {}
        self.joint_names = []
        self.start_time = None
        self.recording = False
        self.should_save = False
        
        # Control flags
        self.running = True
        self.plot_ready = False
        self.shutdown_requested = False
        
        # Display settings
        self.display_window = 30.0  # Show last 30 seconds
        
        # Threading for plot
        self.data_lock = threading.Lock()
        
        self.get_logger().info("Velocity plotter node started and ready.")
    
    def status_callback(self, msg):
        """Callback for movement status messages"""
        if msg.data == 'start':
            self.get_logger().info("Start signal received. Beginning data recording.")
            with self.data_lock:
                # Clear previous data to start fresh
                self.time_data = []
                self.velocity_data = {}
                self.joint_names = []  # Reset joint names to force reinitialization
                self.recording = True
                self.start_time = time.time()
                self.get_logger().info("Data recording initialized.")
            
        elif msg.data == 'complete':
            self.get_logger().info(f"Movement complete signal received. Captured {len(self.time_data)} data points.")
            with self.data_lock:
                self.recording = False
                self.should_save = True
                self.shutdown_requested = True

    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        # Only record data when recording is enabled
        if not self.recording or self.start_time is None:
            self.get_logger().debug("Skipping data recording - not active")
            return
            
        if not msg.velocity:
            self.get_logger().warn("Received joint state with no velocity data")
            return
            
        with self.data_lock:
            # Initialize joint names if not done yet
            if not self.joint_names:
                self.joint_names = list(msg.name)
                for joint_name in self.joint_names:
                    self.velocity_data[joint_name] = []
            
            # Calculate time since start
            current_time = time.time() - self.start_time
            
            # Store the data
            self.time_data.append(current_time)
            
            # Store velocity data for each joint
            for i, joint_name in enumerate(msg.name):
                if joint_name in self.velocity_data:
                    if i < len(msg.velocity):
                        velocity = msg.velocity[i]
                        self.velocity_data[joint_name].append(velocity)
                        # Debug: Log significant velocities for lift joint
                        if joint_name == 'lift_lower_joint' and abs(velocity) > 0.001:
                            self.get_logger().info(f"Lift velocity: {velocity:.6f} rad/s")
                    else:
                        self.velocity_data[joint_name].append(0.0)
    
    def setup_plot(self):
        """Setup the matplotlib plot"""
        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.ax.set_title('Live Joint Velocities vs Time', fontsize=16, fontweight='bold')
        self.ax.set_xlabel('Time (seconds)', fontsize=12)
        self.ax.set_ylabel('Velocity (rad/s)', fontsize=12)
        self.ax.grid(True, alpha=0.3)
        
        # Colors for different joints
        self.colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4', '#FFEAA7', '#DDA0DD', '#FFB6C1', '#98FB98']
        self.lines = {}
        
        # Set initial plot limits
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(-1, 1)
        
        plt.tight_layout()
        self.plot_ready = True

    def animate(self, frame):
        """Animation function for matplotlib"""
        if self.shutdown_requested:
            plt.close(self.fig)
            return []

        if not self.plot_ready:
            return []
            
        with self.data_lock:
            if len(self.time_data) < 2:
                return list(self.lines.values())
            
            # Get recent data for display
            current_time = self.time_data[-1] if self.time_data else 0
            start_time = max(0, current_time - self.display_window)
            
            # Find start index for display window
            start_idx = 0
            for i, t in enumerate(self.time_data):
                if t >= start_time:
                    start_idx = i
                    break
            
            display_time = self.time_data[start_idx:] if self.time_data else []
            
            if not display_time:
                return list(self.lines.values())
            
            # Update lines for each joint
            lines_to_return = []
            legend_labels = []
            
            for i, joint_name in enumerate(self.velocity_data.keys()):
                if joint_name not in self.velocity_data:
                    continue
                    
                velocities = self.velocity_data[joint_name][start_idx:] if len(self.velocity_data[joint_name]) > start_idx else []
                
                if len(velocities) == len(display_time) and velocities:
                    color = self.colors[i % len(self.colors)]
                    
                    if joint_name not in self.lines:
                        line, = self.ax.plot(display_time, velocities, 
                                           color=color, linewidth=2, alpha=0.8, label=joint_name)
                        self.lines[joint_name] = line
                    else:
                        self.lines[joint_name].set_data(display_time, velocities)
                    
                    legend_labels.append(joint_name)
                    lines_to_return.append(self.lines[joint_name])
            
            # Update plot limits
            if display_time:
                self.ax.set_xlim(start_time, current_time + 1)
                
                # Auto-scale y-axis
                all_velocities = []
                for joint_name in self.velocity_data:
                    if joint_name in self.velocity_data:
                        joint_velocities = self.velocity_data[joint_name][start_idx:]
                        if joint_velocities:
                            all_velocities.extend(joint_velocities)
                
                if all_velocities:
                    y_min = min(all_velocities)
                    y_max = max(all_velocities)
                    y_margin = max(0.1, abs(y_max - y_min) * 0.1)
                    self.ax.set_ylim(y_min - y_margin, y_max + y_margin)
            
            # Update legend
            if legend_labels:
                self.ax.legend(legend_labels, loc='upper right', fontsize=10)
            
            return lines_to_return

    def run_plot(self):
        """Run the live plot with ROS spinning in background"""
        # Setup the plot
        self.setup_plot()
        
        # Start ROS spinning in a separate thread
        def ros_spin():
            while rclpy.ok() and self.running and not self.shutdown_requested:
                try:
                    rclpy.spin_once(self, timeout_sec=0.1)
                except Exception as e:
                    self.get_logger().error(f"Error in ROS spinning: {e}")
                    break
            
            self.get_logger().info("ROS spinning has stopped.")


        self.ros_thread = threading.Thread(target=ros_spin, daemon=True)
        self.ros_thread.start()
        
        # This function will be called when the plot window is closed
        def on_close(event):
            self.get_logger().info("Plot window closed by user.")
            self.shutdown_requested = True

        self.fig.canvas.mpl_connect('close_event', on_close)

        # Start the animation
        self.ani = animation.FuncAnimation(
            self.fig, self.animate, interval=100, blit=False, cache_frame_data=False
        )
        
        try:
            # Show the plot - this will block until window is closed
            plt.show()
        except Exception as e:
            self.get_logger().error(f"Error showing plot: {e}")
        finally:
            self.get_logger().info("Plot window closed, ensuring shutdown...")
            self.running = False
            self.ros_thread.join(timeout=1.0) # Wait for ros thread to finish
            # Save plot when window closes
            if self.should_save and self.time_data:
                self.save_plot()
    
    def save_plot(self):
        """Save the final plot to a file"""
        try:
            # Only save if we have data
            if not self.time_data:
                self.get_logger().warn("No data to save")
                return
                
            self.get_logger().info(f"Saving plot with {len(self.time_data)} data points...")
            
            # Create the plot
            plt.style.use('dark_background')
            fig, ax = plt.subplots(figsize=(12, 8))
            ax.set_title('Joint Velocities vs Time', fontsize=16, fontweight='bold')
            ax.set_xlabel('Time (seconds)', fontsize=12)
            ax.set_ylabel('Velocity (rad/s)', fontsize=12)
            ax.grid(True, alpha=0.3)
            
            # Colors for different joints
            colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4', '#FFEAA7', '#DDA0DD', '#FFB6C1', '#98FB98']
            
            # Plot each joint's velocity
            plotted_joints = []
            for i, joint_name in enumerate(self.velocity_data.keys()):
                velocities = self.velocity_data[joint_name]
                if len(velocities) == len(self.time_data) and velocities:
                    color = colors[i % len(colors)]
                    ax.plot(self.time_data, velocities, 
                           color=color, linewidth=2, alpha=0.8, label=joint_name)
                    plotted_joints.append(joint_name)
            
            # Set plot limits
            if self.time_data:
                ax.set_xlim(0, max(self.time_data) + 0.5)
                
                # Auto-scale y-axis
                all_velocities = []
                for velocities in self.velocity_data.values():
                    if velocities:
                        all_velocities.extend(velocities)
                
                if all_velocities:
                    y_min = min(all_velocities)
                    y_max = max(all_velocities)
                    y_margin = max(0.1, abs(y_max - y_min) * 0.1)
                    ax.set_ylim(y_min - y_margin, y_max + y_margin)
            
            # Add legend
            if plotted_joints:
                ax.legend(loc='upper right', fontsize=10)
            
            plt.tight_layout()
            
            # Save with timestamp to avoid overwriting
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f'velocity_plot_{timestamp}.png'
            fig.savefig(filename, dpi=300, bbox_inches='tight')
            plt.close(fig)  # Close to free memory
            
            self.get_logger().info(f"Plot saved to {filename}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save plot: {e}")


def main():
    rclpy.init()
    velocity_plotter = None
    
    try:
        velocity_plotter = VelocityPlotter()

        def signal_handler(signum, frame):
            velocity_plotter.get_logger().info(f"Signal {signum} received, shutting down.")
            velocity_plotter.shutdown_requested = True

        signal.signal(signal.SIGTERM, signal_handler)
        signal.signal(signal.SIGINT, signal_handler)
        
        velocity_plotter.run_plot()

    except KeyboardInterrupt:
        print("\nKeyboard interrupt, shutting down...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if velocity_plotter is not None:
            velocity_plotter.running = False
            # Final save check
            if velocity_plotter.should_save and velocity_plotter.time_data:
                velocity_plotter.save_plot()
            if rclpy.ok():
                velocity_plotter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Velocity plotter shut down successfully.")



if __name__ == '__main__':
    main() 