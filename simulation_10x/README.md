# Cartesian Motion with Trapezoidal Velocity Profiles

This package implements **constant-velocity Cartesian motion** of the end-effector in a 2D plane using **trapezoidal velocity profiles** for the UR10e robot with MoveIt2.

## Features

- **Trapezoidal Velocity Profiles**: Smooth acceleration, constant velocity, and deceleration phases
- **2D Cartesian Motion**: Motion planning in the XY plane with configurable trajectories
- **Real-time Velocity Tracking**: Monitor end-effector and joint velocities during execution
- **Comprehensive Plotting**: Generate detailed analysis plots of velocities and trajectories
- **Multiple Trajectory Types**: Support for linear, circular, and square path patterns
- **MoveIt2 Integration**: Full integration with your existing MoveIt2 setup

## Implementation Overview

### Trapezoidal Velocity Profile

The implementation uses analytical trapezoidal velocity profiles with three phases:

1. **Acceleration Phase**: Linear velocity increase from 0 to max velocity
2. **Constant Velocity Phase**: Maintain maximum velocity
3. **Deceleration Phase**: Linear velocity decrease back to 0

For short distances, the profile automatically becomes triangular (no constant velocity phase).

### Key Parameters

- `max_velocity`: Maximum end-effector velocity (m/s)
- `max_acceleration`: Maximum end-effector acceleration (m/s²)
- `planning_frequency`: Trajectory point generation frequency (Hz)

## Scripts Overview

### 1. Basic Cartesian Motion Planner (`cartesian_motion_planner.py`)

Simple implementation with:
- Basic trapezoidal velocity profile generation
- 2D linear trajectory planning
- Simulation mode for testing without robot
- Basic velocity plotting

### 2. Advanced Cartesian Motion Executor (`advanced_cartesian_motion.py`)

Full-featured implementation with:
- Enhanced trapezoidal profile with analytical calculations
- Multiple trajectory types (line, circle, square)
- Real-time joint state monitoring
- Comprehensive velocity analysis and plotting
- Full MoveIt2 integration with IK/FK services

### 3. Demo Trapezoidal Profile (`demo_trapezoidal_profile.py`)

Standalone demonstration script that:
- Shows different velocity profile types
- Demonstrates 2D Cartesian motion theory
- Generates analysis plots without requiring ROS2/MoveIt
- Educational tool for understanding trapezoidal profiles

## Installation and Setup

### Prerequisites

Ensure your MoveIt2 setup is working correctly:

```bash
# Source your ROS2 workspace
source ~/Code/ws/install/setup.zsh

# Test your MoveIt setup (should work perfectly as mentioned)
ros2 launch simulation_10x moveit_ur10e.launch.py
```

### Build the Package

```bash
cd ~/Code/ws
colcon build --packages-select simulation_10x
source install/setup.zsh
```

## Usage

### 1. Run the Demo (No Robot Required)

First, test the trapezoidal velocity profiles:

```bash
cd ~/Code/ws/src/simulation_10x
python3 scripts/demo_trapezoidal_profile.py
```

This generates:
- `trapezoidal_profile_analysis.png`: Analysis of different velocity profiles
- `2d_cartesian_motion_demo.png`: 2D motion demonstration

### 2. Basic Cartesian Motion with MoveIt

Launch MoveIt2 and the basic motion planner:

```bash
# Terminal 1: Launch MoveIt2
ros2 launch simulation_10x moveit_ur10e.launch.py

# Terminal 2: Run basic Cartesian motion
ros2 run simulation_10x cartesian_motion_planner.py
```

### 3. Advanced Cartesian Motion with Multiple Trajectories

```bash
# Terminal 1: Launch MoveIt2
ros2 launch simulation_10x moveit_ur10e.launch.py

# Terminal 2: Run advanced motion executor
ros2 run simulation_10x advanced_cartesian_motion.py
```

The advanced script executes three different trajectory types:
- **Linear**: Straight line from start to end
- **Circular**: Half-circle path
- **Square**: Square perimeter path

### 4. Launch with Custom Parameters

```bash
# Launch with custom velocity and acceleration
ros2 launch simulation_10x cartesian_motion.launch.py \
    motion_type:=advanced \
    max_velocity:=0.2 \
    max_acceleration:=0.4
```

## Generated Plots and Analysis

The scripts generate comprehensive analysis plots:

### Velocity Profile Analysis
- Position vs Time
- Velocity vs Time (showing trapezoidal shape)
- Acceleration vs Time
- Velocity vs Position
- Phase distribution pie chart

### End-Effector Analysis
- 2D trajectory path with velocity vectors
- X, Y, Z velocity components vs time
- Speed profile showing trapezoidal behavior
- 3D trajectory visualization

### Joint Analysis
- Joint positions vs time
- Joint velocities vs time
- Joint-space trajectory analysis

## Configuration

### Motion Parameters

Edit the scripts to modify motion parameters:

```python
# In the script __init__ method
self.max_velocity = 0.15      # m/s
self.max_acceleration = 0.3   # m/s²
self.planning_freq = 50.0     # Hz
```

### Trajectory Waypoints

Modify start and end poses in the `main()` function:

```python
start_pose = Pose()
start_pose.position.x = 0.4
start_pose.position.y = -0.2
start_pose.position.z = 0.6

end_pose = Pose()
end_pose.position.x = 0.6
end_pose.position.y = 0.2
end_pose.position.z = 0.6
```

## Technical Details

### Velocity Profile Mathematics

The trapezoidal profile is calculated analytically:

```
For distance D, max velocity V, and acceleration A:

if 2 * (V²/2A) >= D:  # Triangular profile
    t_accel = sqrt(D/A)
    actual_max_vel = A * t_accel
    t_constant = 0
else:  # Trapezoidal profile
    t_accel = V/A
    t_constant = (D - V²/A) / V
    actual_max_vel = V

t_total = 2 * t_accel + t_constant
```

### 2D Motion Implementation

Cartesian motion is achieved by:
1. Computing distance and direction vector
2. Generating trapezoidal velocity profile for the distance
3. Interpolating positions along the direction vector
4. Converting to joint space via inverse kinematics
5. Executing joint trajectory with MoveIt2

## Troubleshooting

### Common Issues

1. **MoveIt2 not responding**: Ensure all MoveIt2 services are running
2. **IK failures**: Check that target poses are within robot workspace
3. **Plot display issues**: Ensure matplotlib backend is properly configured
4. **Permission errors**: Verify scripts have execute permissions

### Debug Mode

Enable debug logging:

```python
self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
```

## Results and Validation

The implementation provides:

- **Smooth Motion**: Trapezoidal profiles ensure smooth acceleration/deceleration
- **Constant Speed**: Achieves desired constant velocity in straight-line segments
- **Accurate Tracking**: Real-time monitoring validates velocity profiles
- **Comprehensive Analysis**: Detailed plots verify theoretical vs actual performance

## Files Generated

After running the scripts, you'll find:
- `cartesian_motion_analysis.png`: Basic motion analysis
- `line_trajectory_analysis.png`: Linear trajectory analysis
- `circle_trajectory_analysis.png`: Circular trajectory analysis  
- `square_trajectory_analysis.png`: Square trajectory analysis
- `trapezoidal_profile_analysis.png`: Velocity profile theory
- `2d_cartesian_motion_demo.png`: 2D motion demonstration

## Robot Position Movement Script

The `move_to_positions.py` script allows you to move the UR10e robot to predefined positions from the MoveIt configuration.

### Available Predefined Positions

The script includes all named positions from the SRDF configuration:

1. **home** - Standard home position
2. **extended_reach** - Extended reach configuration
3. **intermediate_workspace** - Intermediate workspace position
4. **singularity_avoidance** - Position to avoid singularities
5. **high_elbow** - High elbow configuration
6. **lateral_extension** - Lateral extension position
7. **rapid_transit** - Position optimized for rapid transit
8. **test_position** - Test configuration position

### Usage

#### Running the Script Directly

```bash
# Move through all positions with 3-second pauses
ros2 run simulation_10x move_to_positions.py all

# Move through all positions with custom pause time (5 seconds)
ros2 run simulation_10x move_to_positions.py all 5.0

# Move to a specific position
ros2 run simulation_10x move_to_positions.py home
ros2 run simulation_10x move_to_positions.py extended_reach

# List all available positions
ros2 run simulation_10x move_to_positions.py list

# Show current joint values
ros2 run simulation_10x move_to_positions.py current

# Show help/usage
ros2 run simulation_10x move_to_positions.py
```

#### Using the Launch File

```bash
# Launch with default settings (move through all positions)
ros2 launch simulation_10x move_positions_demo.launch.py

# Launch with custom pause time
ros2 launch simulation_10x move_positions_demo.launch.py pause_time:=5.0

# Launch to move to specific position
ros2 launch simulation_10x move_positions_demo.launch.py mode:=home

# Launch to list positions
ros2 launch simulation_10x move_positions_demo.launch.py mode:=list
```

### Prerequisites

Before running the position movement script, ensure:

1. The robot simulation or real robot is running
2. MoveIt is properly configured and running
3. The robot controllers are active

### Interactive Demo

For an easy-to-use interactive demonstration:

```bash
# Launch the interactive demo
ros2 run simulation_10x demo_positions.py
```

This provides a menu-driven interface where you can:
- List all positions
- Move to specific positions interactively
- Run demonstration sequences
- Show current joint values
- Quick test movements

### Safety Features

- Velocity and acceleration are limited to 30% of maximum for safety
- Planning timeout is set to 10 seconds
- Each movement is verified before proceeding to the next
- Comprehensive error handling and logging

## Summary of Available Scripts

This package includes several Python scripts for robot control and demonstration:

1. **`move_to_positions.py`** - Core script for moving the robot to predefined positions
2. **`demo_positions.py`** - Interactive demonstration with menu-driven interface
3. **`velocity_plotter.py`** - Velocity profile analysis and plotting
4. **`cartesian_motion_planner.py`** - Cartesian motion planning with trapezoidal profiles
5. **`advanced_cartesian_motion.py`** - Advanced Cartesian motion capabilities
6. **`demo_trapezoidal_profile.py`** - Trapezoidal velocity profile demonstration

All scripts can be run with:
```bash
ros2 run simulation_10x <script_name>.py [arguments]
```

## Future Enhancements

Potential improvements:
- 3D trajectory planning
- Obstacle avoidance integration
- Real-time trajectory modification
- Multiple robot coordination
- Advanced velocity profiles (S-curve, polynomial)
- Interactive position teaching and saving

---

**Note**: This implementation demonstrates the theoretical and practical aspects of constant-velocity Cartesian motion with trapezoidal velocity profiles, as well as predefined position control, providing both educational value and practical robot control capabilities. 