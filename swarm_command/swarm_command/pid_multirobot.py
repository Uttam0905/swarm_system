#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
from typing import Dict, Optional, List

def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class SimplePID:
    def __init__(self, kp=1.2, ki=0.0, kd=0.25, integral_limit=1.0, deriv_alpha=0.6):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.integral_limit = integral_limit
        self.deriv_alpha = deriv_alpha

        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None
        self.prev_derivative = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None
        self.prev_derivative = 0.0

    def update(self, error: float, now_s: float) -> float:
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = now_s - self.prev_time
            if dt <= 0.0:
                dt = 1e-6

        # integral with clamp
        self.integral += error * dt
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < -self.integral_limit:
            self.integral = -self.integral_limit

        # raw derivative
        if self.prev_error is None or dt == 0.0:
            raw_deriv = 0.0
        else:
            raw_deriv = (error - self.prev_error) / dt

        # EWMA smoothing
        deriv = self.deriv_alpha * self.prev_derivative + (1.0 - self.deriv_alpha) * raw_deriv

        out = self.kp * error + self.ki * self.integral + self.kd * deriv

        # save
        self.prev_error = error
        self.prev_time = now_s
        self.prev_derivative = deriv

        return out

class MultiRobotCommonPID(Node):
    def __init__(self):
        super().__init__('pid_multirobot')

        # ----- PARAMETERS (safe defaults for your robot) -----
        self.declare_parameter('robots', ['robot1','robot2','robot3'])
        self.declare_parameter('control_rate', 20.0)  # Hz
        # Robot geometry (used for derived limits; keep consistent with URDF)
        self.declare_parameter('wheel_separation', 0.35)  # meters
        self.declare_parameter('wheel_diameter', 0.10)    # meters

        # Linear PID
        self.declare_parameter('linear_kp', 1.2)
        self.declare_parameter('linear_ki', 0.0)
        self.declare_parameter('linear_kd', 0.0)

        # Angular PID
        self.declare_parameter('angular_kp', 1.6)
        self.declare_parameter('angular_ki', 0.0)
        self.declare_parameter('angular_kd', 0.35)

        # Filters & limits
        self.declare_parameter('deriv_alpha', 0.6)         # derivative EWMA (0..1)
        self.declare_parameter('yaw_filter_alpha', 0.7)    # yaw low-pass (0..1) — higher => smoother
        self.declare_parameter('integral_limit', 1.0)

        # safe clamping & tolerances
        self.declare_parameter('max_linear', 0.6)          # m/s (conservative for given robot)
        self.declare_parameter('max_angular', 1.0)         # rad/s (conservative)
        self.declare_parameter('pos_tol', 0.06)            # meters
        self.declare_parameter('ang_tol', 0.03)            # radians (~1.7°)
        self.declare_parameter('min_ang_cmd', 0.02)        # rad/s deadband to avoid chattering

        # debug
        self.declare_parameter('debug_topic', False)       # publish a debug topic if True

        # ----- READ PARAMS -----
        self.robots: List[str] = self.get_parameter('robots').value
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.wheel_sep = float(self.get_parameter('wheel_separation').value)
        self.wheel_diam = float(self.get_parameter('wheel_diameter').value)

        self.linear_kp = float(self.get_parameter('linear_kp').value)
        self.linear_ki = float(self.get_parameter('linear_ki').value)
        self.linear_kd = float(self.get_parameter('linear_kd').value)

        self.angular_kp = float(self.get_parameter('angular_kp').value)
        self.angular_ki = float(self.get_parameter('angular_ki').value)
        self.angular_kd = float(self.get_parameter('angular_kd').value)

        self.deriv_alpha = float(self.get_parameter('deriv_alpha').value)
        self.yaw_alpha = float(self.get_parameter('yaw_filter_alpha').value)
        self.integral_limit = float(self.get_parameter('integral_limit').value)

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.pos_tol = float(self.get_parameter('pos_tol').value)
        self.ang_tol = float(self.get_parameter('ang_tol').value)
        self.min_ang_cmd = float(self.get_parameter('min_ang_cmd').value)
        self.debug_topic = bool(self.get_parameter('debug_topic').value)

        # Derived safe angular from wheel geometry (informational only)
        # theoretical max omega if wheels at max_linear: omega = 2*v / wheel_sep
        derived_max_omega = 2.0 * self.max_linear / max(0.001, self.wheel_sep)
        self.get_logger().info(f'PID NODE: derived_max_omega={derived_max_omega:.2f} rad/s (wheel_sep={self.wheel_sep})')

        # ----- State containers -----
        self.goals: Dict[str, Dict] = {}
        self.pose: Dict[str, Dict] = {}
        self.enabled: Dict[str, bool] = {}
        self.cmd_pubs: Dict[str, any] = {}
        self.yaw_filtered: Dict[str, Optional[float]] = {}
        self.pid_lin: Dict[str, SimplePID] = {}
        self.pid_ang: Dict[str, SimplePID] = {}

        # initialize per-robot
        for r in self.robots:
            self.goals[r] = {'x': None, 'y': None, 'theta': None}
            self.pose[r] = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            self.enabled[r] = True
            self.yaw_filtered[r] = None

            # publishers / subscribers
            self.cmd_pubs[r] = self.create_publisher(Twist, f'/{r}/cmd_vel', 10)
            self.create_subscription(Pose2D, f'/{r}/goal', self.make_goal_cb(r), 10)
            self.create_subscription(Odometry, f'/{r}/odom', self.make_odom_cb(r), 20)
            self.create_subscription(Bool, f'/{r}/pid_enable', self.make_enable_cb(r), 10)

            # PID objects (same gains for all robots)
            lin_pid = SimplePID(kp=self.linear_kp, ki=self.linear_ki, kd=self.linear_kd,
                                integral_limit=self.integral_limit, deriv_alpha=self.deriv_alpha)
            ang_pid = SimplePID(kp=self.angular_kp, ki=self.angular_ki, kd=self.angular_kd,
                                integral_limit=self.integral_limit, deriv_alpha=self.deriv_alpha)
            self.pid_lin[r] = lin_pid
            self.pid_ang[r] = ang_pid

            self.get_logger().info(f'Initialized PID for {r}')

        # optional debug publisher
        if self.debug_topic:
            from std_msgs.msg import String
            self.debug_pub = self.create_publisher(String, '/pid_debug', 5)

        # control loop timer
        period = 1.0 / max(1.0, self.control_rate)
        self.create_timer(period, self.control_timer_cb)
        self.get_logger().info(f'pid_multirobot started for robots {self.robots} at {self.control_rate} Hz')

    # ---------- callbacks ----------
    def make_goal_cb(self, robot):
        def cb(msg: Pose2D):
            self.goals[robot]['x'] = float(msg.x)
            self.goals[robot]['y'] = float(msg.y)
            self.goals[robot]['theta'] = float(msg.theta) if msg is not None else None
            self.pid_lin[robot].reset()
            self.pid_ang[robot].reset()
            self.yaw_filtered[robot] = None
            self.get_logger().debug(f'[{robot}] new goal {self.goals[robot]}')
        return cb

    def make_odom_cb(self, robot):
        def cb(msg: Odometry):
            self.pose[robot]['x'] = msg.pose.pose.position.x
            self.pose[robot]['y'] = msg.pose.pose.position.y
            raw_yaw = yaw_from_quat(msg.pose.pose.orientation)
            # unwrap + low-pass
            if self.yaw_filtered[robot] is None:
                self.yaw_filtered[robot] = raw_yaw
            else:
                diff = ((raw_yaw - self.yaw_filtered[robot] + math.pi) % (2.0*math.pi)) - math.pi
                # low-pass update: new = old + (1-alpha) * diff
                self.yaw_filtered[robot] = self.yaw_filtered[robot] + (1.0 - self.yaw_alpha) * diff
            self.pose[robot]['yaw'] = self.yaw_filtered[robot]
        return cb

    def make_enable_cb(self, robot):
        def cb(msg: Bool):
            self.enabled[robot] = bool(msg.data)
            if not self.enabled[robot]:
                # publish zero immediately and reset controllers
                zero = Twist()
                self.cmd_pubs[robot].publish(zero)
                self.pid_lin[robot].reset()
                self.pid_ang[robot].reset()
            self.get_logger().info(f'[{robot}] pid_enable={self.enabled[robot]}')
        return cb

    @staticmethod
    def shortest_angular_diff(target, source):
        a = target - source
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    # ---------- control ----------
    def control_timer_cb(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        for r in self.robots:
            gx = self.goals[r]['x']; gy = self.goals[r]['y']; gtheta = self.goals[r]['theta']
            if gx is None or gy is None:
                continue

            dx = gx - self.pose[r]['x']; dy = gy - self.pose[r]['y']
            dist = math.hypot(dx, dy)

            # desired yaw logic
            if dist > 0.12:
                desired_yaw = math.atan2(dy, dx)
            else:
                desired_yaw = gtheta if gtheta is not None else math.atan2(dy, dx)

            yaw_err = self.shortest_angular_diff(desired_yaw, self.pose[r]['yaw'])

            pos_reached = dist <= self.pos_tol
            ang_target = gtheta if gtheta is not None else desired_yaw
            ang_reached = abs(self.shortest_angular_diff(ang_target, self.pose[r]['yaw'])) <= self.ang_tol

            # orientation-only control if position reached but angle not
            if pos_reached and (gtheta is not None) and (not ang_reached):
                ang_err = self.shortest_angular_diff(gtheta, self.pose[r]['yaw'])
                ang_out = self.pid_ang[r].update(ang_err, now_s)
                ang_cmd = max(-self.max_angular, min(self.max_angular, ang_out))
                # deadband
                if abs(ang_cmd) < self.min_ang_cmd:
                    ang_cmd = 0.0
                    self.pid_ang[r].prev_derivative = 0.0
                if self.enabled[r]:
                    t = Twist(); t.linear.x = 0.0; t.angular.z = ang_cmd
                    self.cmd_pubs[r].publish(t)
                continue

            # normal control
            lin_out = self.pid_lin[r].update(dist, now_s)
            ang_out = self.pid_ang[r].update(yaw_err, now_s)

            linear_vel = max(-self.max_linear, min(self.max_linear, lin_out))
            angular_vel = max(-self.max_angular, min(self.max_angular, ang_out))

            # scale forward with yaw error
            if abs(yaw_err) > 1.0:
                linear_vel = 0.0
            else:
                linear_vel = linear_vel * max(0.0, math.cos(yaw_err))

            # angular deadband
            if abs(angular_vel) < self.min_ang_cmd:
                angular_vel = 0.0
                self.pid_ang[r].prev_derivative = 0.0

            if not self.enabled[r]:
                continue

            t = Twist(); t.linear.x = linear_vel; t.angular.z = angular_vel
            self.cmd_pubs[r].publish(t)

            if self.debug_topic:
                try:
                    msg = f"{r}: dist={dist:.3f}, yaw_err={yaw_err:.3f}, lin={linear_vel:.3f}, ang={angular_vel:.3f}"
                    self.debug_pub.publish(msg)
                except Exception:
                    pass

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotCommonPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
