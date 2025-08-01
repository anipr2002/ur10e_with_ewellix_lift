#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    PlanningOptions,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    WorkspaceParameters,
    CartesianTrajectory,
    CartesianTrajectoryPoint
)
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
import numpy as np
import math
import time


class CartesianShapeDrawer(Node):
    def __init__(self):
        super().__init__('cartesian_shape_drawer')
        
        # Create action clients for MoveIt2
        self.ur_action_client = ActionClient(self, MoveGroup, '/move_action')
        self.lift_action_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Wait for action servers
        self.get_logger().info("Waiting for MoveGroup action servers...")
        self.ur_action_client.wait_for_server()
        self.lift_action_client.wait_for_server()
        self.get_logger().info("Connected to MoveIt2 action servers")
        
        # Robot configuration
        self.ur_joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        
        self.lift_joint_names = ["lift_lower_joint"]
        
        # Drawing parameters
        self.drawing_heights = [0.05, 0.10, 0.15]  # Different Z-heights using lift
        self.end_effector_orientation = [0.0, 1.0, 0.0, 0.0]  # Fixed orientation (quaternion)
        self.cartesian_step_size = 0.005  # 5mm steps for smooth paths
        self.velocity_scaling = 0.1  # Constant velocity scaling
        
        # Shape parameters
        self.circle_radius = 0.15  # 15cm radius
        self.circle_center = [0.3, 0.0]  # Center in XY plane relative to base
        self.triangle_size = 0.2  # 20cm side length
        self.triangle_center = [0.5, 0.0]  # Center in XY plane relative to base
        
        self.get_logger().info("CartesianShapeDrawer initialized")
    
    def create_pose(self, x, y, z, qx=0.0, qy=1.0, qz=0.0, qw=0.0):
        """Create a Pose message from position and orientation"""
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)
        return pose
    
    def generate_circle_waypoints(self, center_x, center_y, z_height, radius, num_points=36):
        """Generate waypoints for a circle in the XY plane"""
        waypoints = []
        
        for i in range(num_points + 1):  # +1 to close the circle
            angle = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            
            pose = self.create_pose(x, y, z_height, *self.end_effector_orientation)
            waypoints.append(pose)
        
        return waypoints
    
    def generate_triangle_waypoints(self, center_x, center_y, z_height, size):
        """Generate waypoints for an equilateral triangle in the XY plane"""
        waypoints = []
        
        # Calculate triangle vertices (equilateral triangle)
        height = size * math.sqrt(3) / 2
        vertices = [
            [center_x, center_y + 2*height/3],           # Top vertex
            [center_x - size/2, center_y - height/3],     # Bottom left
            [center_x + size/2, center_y - height/3],     # Bottom right
            [center_x, center_y + 2*height/3]            # Back to top (close triangle)
        ]
        
        # Interpolate between vertices for smooth motion
        for i in range(len(vertices) - 1):
            start_vertex = vertices[i]
            end_vertex = vertices[i + 1]
            
            # Generate intermediate points along each edge
            num_points = 12  # Points per edge
            for j in range(num_points):
                t = j / float(num_points - 1) if num_points > 1 else 0
                x = start_vertex[0] + t * (end_vertex[0] - start_vertex[0])
                y = start_vertex[1] + t * (end_vertex[1] - start_vertex[1])
                
                pose = self.create_pose(x, y, z_height, *self.end_effector_orientation)
                waypoints.append(pose)
        
        return waypoints
    
    def move_lift_to_height(self, height):
        """Move the lift to specified height"""
        self.get_logger().info(f"Moving lift to height: {height}m")
        
        # Create MoveGroup goal
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = "lift"
        goal.request.num_planning_attempts = 10
        goal.request.max_velocity_scaling_factor = self.velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.velocity_scaling
        
        # Set planner
        goal.request.planner_id = "PTP"
        
        # Set goal constraints
        goal.request.goal_constraints = [Constraints()]
        
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = "lift_lower_joint"
        joint_constraint.position = height
        joint_constraint.tolerance_above = 0.01
        joint_constraint.tolerance_below = 0.01
        joint_constraint.weight = 1.0
        
        goal.request.goal_constraints[0].joint_constraints = [joint_constraint]
        
        # Planning options
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5
        
        # Send goal and wait
        future = self.lift_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Lift movement goal rejected")
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("Lift movement completed successfully")
            return True
        else:
            self.get_logger().error(f"Lift movement failed with error code: {result.result.error_code.val}")
            return False
    
    def execute_cartesian_path(self, waypoints, shape_name):
        """Execute Cartesian path by moving to each waypoint sequentially"""
        self.get_logger().info(f"Executing Cartesian path for {shape_name} with {len(waypoints)} waypoints")
        
        success_count = 0
        total_waypoints = len(waypoints)
        
        for i, waypoint in enumerate(waypoints):
            self.get_logger().info(f"Moving to waypoint {i+1}/{total_waypoints}")
            
            # Create MoveGroup goal for this waypoint
            goal = MoveGroup.Goal()
            goal.request = MotionPlanRequest()
            goal.request.group_name = "ur_manipulator"
            goal.request.num_planning_attempts = 5
            goal.request.max_velocity_scaling_factor = self.velocity_scaling
            goal.request.max_acceleration_scaling_factor = self.velocity_scaling
            
            # Use LIN planner for linear motion
            goal.request.planner_id = "LIN"
            
            # Set workspace parameters
            goal.request.workspace_parameters = WorkspaceParameters()
            goal.request.workspace_parameters.header.frame_id = "base_link"
            goal.request.workspace_parameters.min_corner.x = -1.0
            goal.request.workspace_parameters.min_corner.y = -1.0
            goal.request.workspace_parameters.min_corner.z = -0.5
            goal.request.workspace_parameters.max_corner.x = 1.0
            goal.request.workspace_parameters.max_corner.y = 1.0
            goal.request.workspace_parameters.max_corner.z = 1.5
            
            # Set goal constraints
            goal.request.goal_constraints = [Constraints()]
            
            # Position constraint
            position_constraint = PositionConstraint()
            position_constraint.header.frame_id = "base_link"
            position_constraint.link_name = "tool0"
            position_constraint.target_point_offset.x = 0.0
            position_constraint.target_point_offset.y = 0.0
            position_constraint.target_point_offset.z = 0.0
            
            # Define constraint region
            constraint_region = SolidPrimitive()
            constraint_region.type = SolidPrimitive.SPHERE
            constraint_region.dimensions = [0.01]  # 1cm tolerance
            
            position_constraint.constraint_region.primitives = [constraint_region]
            position_constraint.constraint_region.primitive_poses = [waypoint]
            position_constraint.weight = 1.0
            
            # Orientation constraint
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = "base_link"
            orientation_constraint.link_name = "tool0"
            orientation_constraint.orientation = waypoint.orientation
            orientation_constraint.absolute_x_axis_tolerance = 0.1
            orientation_constraint.absolute_y_axis_tolerance = 0.1
            orientation_constraint.absolute_z_axis_tolerance = 0.1
            orientation_constraint.weight = 1.0
            
            goal.request.goal_constraints[0].position_constraints = [position_constraint]
            goal.request.goal_constraints[0].orientation_constraints = [orientation_constraint]
            
            # Planning options
            goal.planning_options = PlanningOptions()
            goal.planning_options.plan_only = False
            goal.planning_options.replan = True
            goal.planning_options.replan_attempts = 3
            
            # Send goal and wait
            future = self.ur_action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f"Waypoint {i+1} goal rejected")
                continue
            
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            result = result_future.result()
            if result.result.error_code.val == 1:  # SUCCESS
                success_count += 1
                self.get_logger().info(f"Waypoint {i+1} completed successfully")
            else:
                self.get_logger().warning(f"Waypoint {i+1} failed with error code: {result.result.error_code.val}")
            
            # Small delay for smooth motion
            time.sleep(0.1)
        
        success_rate = (success_count / total_waypoints) * 100
        self.get_logger().info(f"Cartesian path execution completed. Success rate: {success_rate:.1f}%")
        return success_rate > 70  # Consider successful if >70% waypoints reached
    
    def move_to_start_position(self, start_pose):
        """Move to the starting position of a shape"""
        self.get_logger().info("Moving to start position")
        
        # Create MoveGroup goal
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = "ur_manipulator"
        goal.request.num_planning_attempts = 10
        goal.request.max_velocity_scaling_factor = self.velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.velocity_scaling
        
        # Use PTP planner for initial positioning
        goal.request.planner_id = "PTP"
        
        # Set goal constraints
        goal.request.goal_constraints = [Constraints()]
        
        # Position constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "base_link"
        position_constraint.link_name = "tool0"
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        
        constraint_region = SolidPrimitive()
        constraint_region.type = SolidPrimitive.SPHERE
        constraint_region.dimensions = [0.01]
        
        position_constraint.constraint_region.primitives = [constraint_region]
        position_constraint.constraint_region.primitive_poses = [start_pose]
        position_constraint.weight = 1.0
        
        # Orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.link_name = "tool0"
        orientation_constraint.orientation = start_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0
        
        goal.request.goal_constraints[0].position_constraints = [position_constraint]
        goal.request.goal_constraints[0].orientation_constraints = [orientation_constraint]
        
        # Planning options
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5
        
        # Send goal and wait
        future = self.ur_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Start position goal rejected")
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("Moved to start position successfully")
            return True
        else:
            self.get_logger().error(f"Failed to move to start position. Error code: {result.result.error_code.val}")
            return False
    
    def draw_shapes_at_heights(self):
        """Main function to draw shapes at different heights"""
        self.get_logger().info("Starting Cartesian shape drawing sequence")
        
        shapes = [
            ("circle", self.generate_circle_waypoints),
            ("triangle", self.generate_triangle_waypoints)
        ]
        
        for height in self.drawing_heights:
            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info(f"Drawing at height: {height}m")
            self.get_logger().info(f"{'='*50}")
            
            # Move lift to desired height
            if not self.move_lift_to_height(height):
                self.get_logger().error(f"Failed to move lift to height {height}m")
                continue
            
            # Wait for lift to stabilize
            time.sleep(2.0)
            
            for shape_name, shape_generator in shapes:
                self.get_logger().info(f"\nDrawing {shape_name}...")
                
                # Generate waypoints for the shape
                if shape_name == "circle":
                    waypoints = shape_generator(
                        self.circle_center[0], 
                        self.circle_center[1], 
                        0.3,  # Fixed Z for end-effector (lift handles vertical positioning)
                        self.circle_radius
                    )
                else:  # triangle
                    waypoints = shape_generator(
                        self.triangle_center[0], 
                        self.triangle_center[1], 
                        0.3,  # Fixed Z for end-effector
                        self.triangle_size
                    )
                
                if not waypoints:
                    self.get_logger().error(f"Failed to generate waypoints for {shape_name}")
                    continue
                
                # Move to start position
                if not self.move_to_start_position(waypoints[0]):
                    self.get_logger().error(f"Failed to move to start position for {shape_name}")
                    continue
                
                # Wait for robot to stabilize
                time.sleep(1.0)
                
                # Execute the Cartesian path
                success = self.execute_cartesian_path(waypoints, shape_name)
                
                if success:
                    self.get_logger().info(f"Successfully drew {shape_name} at height {height}m")
                else:
                    self.get_logger().error(f"Failed to complete {shape_name} at height {height}m")
                
                # Wait between shapes
                time.sleep(2.0)
        
        self.get_logger().info("\nCartesian shape drawing sequence completed!")
    
    def run(self):
        """Run the shape drawing sequence"""
        try:
            self.draw_shapes_at_heights()
        except KeyboardInterrupt:
            self.get_logger().info("Shape drawing interrupted by user")
        except Exception as e:
            self.get_logger().error(f"An error occurred: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    
    drawer = CartesianShapeDrawer()
    
    try:
        drawer.run()
    finally:
        drawer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()