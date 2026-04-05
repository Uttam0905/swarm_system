#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import time

class SwarmLogger(Node):
    def __init__(self):
        super().__init__('swarm_logger')
        
        self.r1 = [0.0, 0.0]
        self.r2 = [0.0, 0.0]
        self.r3 = [0.0, 0.0]

        self.file = open('swarm_data.csv', 'w', newline='')
        self.writer = csv.writer(self.file)

        self.writer.writerow([
            'time',
            'r1_x','r1_y',
            'r2_x','r2_y',
            'r3_x','r3_y'
        ])

        self.start_time = time.time()

        self.create_subscription(Odometry, '/robot1/odom', self.cb_r1, 10)
        self.create_subscription(Odometry, '/robot2/odom', self.cb_r2, 10)
        self.create_subscription(Odometry, '/robot3/odom', self.cb_r3, 10)

        self.timer = self.create_timer(0.1, self.log_data)  # 10 Hz

        self.get_logger().info("Swarm logger started...")

    def cb_r1(self, msg):
        self.r1[0] = msg.pose.pose.position.x
        self.r1[1] = msg.pose.pose.position.y

    def cb_r2(self, msg):
        self.r2[0] = msg.pose.pose.position.x
        self.r2[1] = msg.pose.pose.position.y

    def cb_r3(self, msg):
        self.r3[0] = msg.pose.pose.position.x
        self.r3[1] = msg.pose.pose.position.y

    def log_data(self):
        t = time.time() - self.start_time

        self.writer.writerow([
            t,
            self.r1[0], self.r1[1],
            self.r2[0], self.r2[1],
            self.r3[0], self.r3[1]
        ])

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SwarmLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
