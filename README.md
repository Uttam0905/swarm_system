# swarm_system
## Overview

This project implements a ROS 2–based multi-robot swarm coordination system in Gazebo.
Three differential-drive robots are spawned at arbitrary locations and coordinated using formation control and leader–follower behavior, controlled by a common multi-robot PID controller.

The system supports explicit mode switching, LiDAR-based reactive safety, and safe transitions between autonomous and manual control, following real-world robotics system design practices.

## Key Features

### Formation Control

  * Robots form a straight-line formation from random initial positions.

  * Formation goals are computed using the swarm centroid.

### Leader–Follower Control

 * One robot acts as the leader.

 * Followers mirror the leader’s velocity commands.

### Common Multi-Robot PID Controller

 * Single PID implementation shared across all robots.

 * Integral anti-windup, derivative smoothing, and yaw filtering.

 * Runtime enable/disable using /robotX/pid_enable.

### Mode Switching

 * formation – goal-based formation control using PID.

 * follow – velocity-based leader–follower control.

 * stop – emergency stop triggered by LiDAR.

 * reset – robots return autonomously to initial spawn positions.

### LiDAR-Based Reactive Safety

 * Each robot has an independent LiDAR sensor.

 * If any robot detects an obstacle within 1 m, the entire swarm halts.

## Modes of Operation
### Formation Mode

 * Robots compute the swarm centroid.

 * Each robot receives a goal with a fixed lateral offset.

 * Motion is controlled by the shared PID controller.

### Follow Mode

 * PID controllers are disabled.

 * Followers directly receive the leader’s velocity commands.

 * Used for teleoperation-driven swarm motion.

### Stop Mode (Safety)

 * Triggered automatically if any LiDAR detects an obstacle < 1 m.

 * All PID controllers disabled.

 * All robots commanded to zero velocity.

### Reset Mode

 * Robots autonomously return to their initial spawn positions using PID goals.

 * When all robots reach their start poses, the system returns to formation mode.

## How to Run
1. Launch Gazebo and spawn robots
~~~
ros2 launch swarm_robot swarm_gazebo.launch.py
~~~

2. Start the swarm manager
~~~
ros2 launch swarm_command swarm_manager.launch.py
~~~
## Switching Modes
### Formation mode
~~~
ros2 topic pub -1 /swarm_mode std_msgs/String "{data: formation}"
~~~
### Follow mode
~~~
ros2 topic pub -1 /swarm_mode std_msgs/String "{data: follow}"
~~~
### Emergency stop
~~~
ros2 topic pub -1 /swarm_mode std_msgs/String "{data: stop}"
~~~
### Reset to initial positions
~~~
ros2 topic pub -1 /swarm_mode std_msgs/String "{data: reset}"
~~~

## Design Decisions

### Explicit mode switching instead of killing nodes
Ensures safe, deterministic transitions between behaviors.

### Separation of coordination and control
Swarm logic is isolated from PID control for modularity.

### Reactive safety over global planning
Appropriate for formation experiments in controlled environments.

## Limitations

No global path planning or map-based navigation.

Obstacle handling is reactive (stop-only).

Designed for simulation environments.

## Future Work

Sector-based obstacle avoidance instead of full stop.

Distributed swarm coordination without a central manager.

Integration with global mapping and localization pipelines.

## License

MIT License
