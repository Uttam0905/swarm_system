# swarm_system
Overview

This project implements a ROS 2–based multi-robot swarm coordination system in Gazebo.
Three differential-drive robots are spawned at random locations and coordinated using formation control and leader–follower behavior, with a common PID controller shared across all robots.

The system is designed to be modular, reproducible, and safe, with explicit mode switching and LiDAR-based emergency stopping.

Key Features

Leader–Follower Control

One robot acts as the leader.

Followers mirror the leader’s velocity commands in follow mode.

Formation Control

Robots form and maintain a straight-line formation from arbitrary spawn positions.

Formation goals are generated using the swarm centroid.

Common Multi-Robot PID Controller

Single PID implementation shared by all robots.

Integral anti-windup, derivative smoothing, and yaw filtering.

Runtime enable/disable using pid_enable topic.

Mode Switching

formation – robots align using goal-based PID control.

follow – followers track leader velocity.

stop – emergency stop when any robot detects an obstacle.

reset – robots autonomously return to initial spawn positions.

LiDAR-Based Reactive Safety

Each robot monitors its LiDAR scan.

If any robot detects an obstacle within 1 m, the entire swarm halts.

Gazebo Simulation

Fully simulated differential-drive robots with individual LiDAR sensors.

Designed for controlled environments without global navigation assumptions.

Modes of Operation
1. Formation Mode

Robots compute the swarm centroid.

Each robot receives a goal with a fixed lateral offset.

Motion is controlled by the shared PID controller.

2. Follow Mode

PID controllers are disabled.

Followers directly receive the leader’s velocity commands.

Used for teleoperation-driven motion.

3. Stop Mode (Safety)

Triggered automatically if any LiDAR detects an obstacle < 1 m.

All PID controllers disabled.

All robots commanded to zero velocity.

4. Reset Mode

Robots navigate back to their initial spawn positions using PID goals.

Once all robots reach their start poses, the system returns to formation mode.

Topics Used
Topic	Type	Description
''' /swarm_mode	std_msgs/String	Mode switching (formation, follow, stop, reset) '''
/robotX/goal	geometry_msgs/Pose2D	Goal for PID controller
/robotX/pid_enable	std_msgs/Bool	Enable/disable PID
/robotX/cmd_vel	geometry_msgs/Twist	Velocity command
/robotX/odom	nav_msgs/Odometry	Robot odometry
/robotX/scan	sensor_msgs/LaserScan	LiDAR scan
How to Run
1. Launch Gazebo and robots
ros2 launch swarm_robot swarm_gazebo.launch.py

2. Start the common PID controller
ros2 run swarm_command pid_multirobot

3. Start the swarm manager
ros2 run swarm_command swarm_manager

4. Switch modes
# Formation mode
ros2 topic pub -1 /swarm_mode std_msgs/String "{data: formation}"

# Follow mode
ros2 topic pub -1 /swarm_mode std_msgs/String "{data: follow}"

# Emergency stop
ros2 topic pub -1 /swarm_mode std_msgs/String "{data: stop}"

# Reset to initial positions
ros2 topic pub -1 /swarm_mode std_msgs/String "{data: reset}"

Design Decisions

Separation of control and coordination
PID control is isolated from swarm logic for clarity and reuse.

Explicit mode switching instead of node killing
Ensures safe transitions and deterministic behavior.

Reactive safety instead of global navigation
Appropriate for formation experiments in controlled environments.

# Limitations

No global path planning or map-based navigation.

Obstacle handling is reactive (stop-only).

Designed for controlled simulation environments.

# Future Work

Sector-based obstacle avoidance instead of full stop.

Distributed formation control without a central manager.

Integration with global mapping and localization (handled in a separate project).

# License

MIT License
