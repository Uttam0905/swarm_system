#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from typing import Dict, List
import time

def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class SwarmManager(Node):
    def __init__(self):
        super().__init__("swarm_manager")

        self.declare_parameter("robots", ["robot1", "robot2", "robot3"])
        self.declare_parameter("line_offset", 1.0)
        self.robots = list(self.get_parameter("robots").value)
        self.line_offset = float(self.get_parameter("line_offset").value)

        self.odom = {r: {"x": 0.0, "y": 0.0, "yaw": 0.0} for r in self.robots}

        self.start_pose = {r: None for r in self.robots}

        for r in self.robots:
            self.create_subscription(Odometry, f"/{r}/odom", self.make_odom_cb(r), 10)
            self.create_subscription(LaserScan, f"/{r}/scan", self.make_scan_cb(r), 10)

        leader = self.robots[0]
        self.create_subscription(Twist, f"/{leader}/cmd_vel", self.leader_cmd_cb, 10)

        self.create_subscription(String, "/swarm_mode", self.mode_cb, 10)


        self.goal_pubs = {r: self.create_publisher(Pose2D, f"/{r}/goal", 10) for r in self.robots}
        self.pid_enable_pubs = {r: self.create_publisher(Bool, f"/{r}/pid_enable", 10) for r in self.robots}
        self.cmd_pubs = {r: self.create_publisher(Twist, f"/{r}/cmd_vel", 10) for r in self.robots}

        self.mode = "formation"
        self.obstacle_detected = False
        self.last_leader_cmd = Twist()

        self.get_logger().info("Initial mode: formation")

        self.create_timer(0.1, self.control_loop)

    def make_odom_cb(self, robot):
        def cb(msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = yaw_from_quat(msg.pose.pose.orientation)
            self.odom[robot] = {"x": x, "y": y, "yaw": yaw}

            if self.start_pose[robot] is None:
                self.start_pose[robot] = (x, y, yaw)
        return cb

    def make_scan_cb(self, robot):
        def cb(msg: LaserScan):
            if len(msg.ranges) == 0:
                return
            min_range = min(msg.ranges)
            if min_range < 1.0:  
                if self.mode != "stop":  
                    self.get_logger().warn(f"Obstacle detected by {robot}: {min_range:.2f}m — entering STOP mode")
                self.obstacle_detected = True
                self.mode = "stop"
        return cb

    def leader_cmd_cb(self, msg: Twist):
        self.last_leader_cmd = msg

    def mode_cb(self, msg: String):
        new_mode = msg.data.strip().lower()
        if new_mode not in ["formation", "follow", "stop", "reset"]:
            self.get_logger().warn(f"Unknown mode {new_mode}")
            return

        if new_mode == "stop":
            self.enter_stop_mode()
            return

        if new_mode == "reset":
            self.enter_reset_mode()
            return

        self.transition_mode(new_mode)


    def transition_mode(self, new_mode):
        if new_mode == self.mode:
            return

        self.get_logger().info(f"Mode {self.mode} -> {new_mode}")

        if new_mode == "follow":
            for r in self.robots:
                self.pid_enable_pubs[r].publish(Bool(data=False))
            time.sleep(0.2)
            for r in self.robots:
                self.cmd_pubs[r].publish(Twist())

        if new_mode == "formation":
            for r in self.robots:
                self.pid_enable_pubs[r].publish(Bool(data=True))
            time.sleep(0.05)

        self.mode = new_mode

    def enter_stop_mode(self):
        self.mode = "stop"
        self.get_logger().warn("SWITCHING TO STOP MODE — OBSTACLE DETECTED")

        for r in self.robots:
            self.pid_enable_pubs[r].publish(Bool(data=False))

        zero = Twist()
        for r in self.robots:
            self.cmd_pubs[r].publish(zero)

    def enter_reset_mode(self):
        self.get_logger().info("RESET MODE: returning robots to spawn positions")

        for r in self.robots:
            self.pid_enable_pubs[r].publish(Bool(data=True))

        self.mode = "reset"

    def compute_average(self):
        xs = [self.odom[r]["x"] for r in self.robots]
        ys = [self.odom[r]["y"] for r in self.robots]
        return sum(xs)/len(xs), sum(ys)/len(ys)


    def control_loop(self):
        if self.mode == "stop":
            for r in self.robots:
                self.cmd_pubs[r].publish(Twist())
            return

        if self.mode == "reset":
            all_reached = True
            for r in self.robots:
                sx, sy, syaw = self.start_pose[r]
                g = Pose2D(x=sx, y=sy, theta=0.0)
                self.goal_pubs[r].publish(g)

                dist = math.hypot(self.odom[r]["x"] - sx, self.odom[r]["y"] - sy)
                if dist > 0.1:
                    all_reached = False

            if all_reached:
                self.get_logger().info("RESET complete — switching to FORMATION")
                self.mode = "formation"
            return

        if self.mode == "formation":
            avg_x, avg_y = self.compute_average()
            offsets = [0.0, self.line_offset, -self.line_offset]
            for r, off in zip(self.robots, offsets):
                g = Pose2D(x=avg_x, y=avg_y + off, theta=0.0)
                self.goal_pubs[r].publish(g)
            return

        if self.mode == "follow":
            leader_cmd = self.last_leader_cmd
            for r in self.robots[1:]:
                self.cmd_pubs[r].publish(leader_cmd)

def main():
    rclpy.init()
    node = SwarmManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
