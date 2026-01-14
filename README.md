# swarm_system
Overview

This project implements a ROS 2–based multi-robot swarm coordination system in Gazebo.
Three differential-drive robots are spawned at arbitrary locations and coordinated using formation control and leader–follower behavior, controlled by a common multi-robot PID controller.

The system supports explicit mode switching, LiDAR-based reactive safety, and safe transitions between autonomous and manual control, following real-world robotics system design practices.

Key Features

Formation Control

Robots form a straight-line formation from random initial positions.

Formation goals are computed using the swarm centroid.

Leader–Follower Control

One robot acts as the leader.

Followers mirror the leader’s velocity commands.

Common Multi-Robot PID Controller

Single PID implementation shared across all robots.

Integral anti-windup, derivative smoothing, and yaw filtering.

Runtime enable/disable using /robotX/pid_enable.

Mode Switching

formation – goal-based formation control using PID.

follow – velocity-based leader–follower control.

stop – emergency stop triggered by LiDAR.

reset – robots return autonomously to initial spawn positions.

LiDAR-Based Reactive Safety

Each robot has an independent LiDAR sensor.

If any robot detects an obstacle within 1 m, the entire swarm halts.

Gazebo Simulation

2. Start the common multi-robot PID controller
ros2 run swarm_command pid_multirobot

3. Start the swarm manager
ros2 run swarm_command swarm_manager

Differential-drive robots with individual namespaces.

Fully reproducible simulation environment.
