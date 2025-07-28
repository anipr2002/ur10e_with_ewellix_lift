#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from std_msgs.msg import String
import time
import subprocess
import os
import signal


class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Positions from SRDF
        self.positions = {
            "home": [0.0, -1.0067, 0.9546, -1.215, -1.6315, 0.0],
            "extended_reach": [1.57, -0.78, -1.57, -1.57, 1.57, 0.0],
            "intermediate_workspace": [0.78, -1.05, 0.78, -0.52, -0.78, 1.57],
            "singularity_avoidance": [-1.57, -2.09, 1.05, -0.52, 1.57, 3.14],
            "high_elbow": [0.52, -0.52, -2.09, -1.05, -1.05, 2.09],
            "lateral_extension": [2.36, -1.31, 0.0, -2.09, 0.0, -1.57],
            "rapid_transit": [-0.78, -1.83, 2.09, 0.52, 0.78, 4.71],
            "test_position": [0.0, -1.0067, 0.9546, -1.215, -1.6315, 0.0]
        }
        
        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        
        # Publisher for movement status
        self.status_publisher = self.create_publisher(String, '/movement_status', 10)
        
        # Initialize velocity plotter process variable
        self.velocity_plotter_process = None
        
        print("Waiting for MoveGroup action server...")
        self._action_client.wait_for_server()
        print("Connected!")

    def start_velocity_plotter(self):
        """Start the velocity plotter as a subprocess"""
        try:
            # Get the directory of the current script
            script_dir = os.path.dirname(os.path.abspath(__file__))
            velocity_plotter_path = os.path.join(script_dir, 'velocity_plotter.py')
            
            print("Starting velocity plotter...")
            # Start velocity plotter as subprocess
            self.velocity_plotter_process = subprocess.Popen(
                ['python3', velocity_plotter_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            print(f"Velocity plotter started with PID: {self.velocity_plotter_process.pid}")
            # Give the plotter a moment to initialize
            time.sleep(2)
            
        except Exception as e:
            print(f"Failed to start velocity plotter: {e}")
            self.velocity_plotter_process = None

    def stop_velocity_plotter(self):
        """Stop the velocity plotter subprocess"""
        if self.velocity_plotter_process and self.velocity_plotter_process.poll() is None:
            print("Stopping velocity plotter...")
            try:
                # Send SIGTERM for graceful shutdown
                self.velocity_plotter_process.terminate()
                # Wait for process to terminate
                self.velocity_plotter_process.wait(timeout=10)
                print("Velocity plotter stopped successfully")
            except subprocess.TimeoutExpired:
                print("Velocity plotter didn't stop gracefully, forcing shutdown...")
                self.velocity_plotter_process.kill()
                self.velocity_plotter_process.wait()
            except Exception as e:
                print(f"Error stopping velocity plotter: {e}")
            finally:
                self.velocity_plotter_process = None

    def publish_start_signal(self):
        """Publish a start signal for coordination"""
        status_msg = String()
        status_msg.data = 'start'
        self.status_publisher.publish(status_msg)
        print("Published start status.")

    def publish_complete_signal(self):
        """Publish completion signal"""
        status_msg = String()
        status_msg.data = 'complete'
        self.status_publisher.publish(status_msg)
        print("Published completion status.")
        # Give time for the message to be received
        time.sleep(1)

    def move_to_position(self, name, joint_values):
        goal = MoveGroup.Goal()
        goal.request.group_name = "ur_manipulator"
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        
        constraints = Constraints()
        for joint_name, value in zip(self.joint_names, joint_values):
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = value
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        
        goal.request.goal_constraints.append(constraints)
        
        print(f"Moving to {name}...")
        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()
            
            if result.result.error_code.val == 1:
                print(f"Reached {name}")
                return True
            else:
                print(f"Failed to reach {name}")
                return False
        else:
            print(f"Goal rejected for {name}")
            return False

    def run_sequence(self):
        print("Starting robot movement sequence...")
        
        # Start velocity plotter
        self.start_velocity_plotter()
        
        # Publish start signal
        self.publish_start_signal()
        
        try:
            print("Beginning position sequence...")
            for name, joint_values in self.positions.items():
                if self.move_to_position(name, joint_values):
                    time.sleep(1)
                else:
                    print(f"Stopping sequence due to failure at {name}")
                    break
            print("Sequence complete!")
            
        finally:
            # Always publish completion and stop plotter, even if movement failed
            self.publish_complete_signal()
            # Give the velocity plotter time to save the plot
            time.sleep(3)
            self.stop_velocity_plotter()


def main():
    rclpy.init()
    mover = None
    try:
        mover = RobotMover()
        mover.run_sequence()
    except KeyboardInterrupt:
        print("\nReceived keyboard interrupt, shutting down...")
    except Exception as e:
        print(f"Error during execution: {e}")
    finally:
        if mover:
            # Ensure velocity plotter is stopped
            mover.stop_velocity_plotter()
            mover.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 