#!/usr/bin/env python3
"""
SICNav <-> ROS 2 Bridge Node
Coordinates between ROS 2 topics and the SICNav policy planner:
- Subscribes: /odom, /crowd/human_states, /goal_pose, /scan
- Publishes: /cmd_vel (to drive TurtleBot 4), /sicnav/status
"""

import math
import sys
import os
import json
import time

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    HAVE_ROS2 = True
except ImportError:
    HAVE_ROS2 = False

from .policy_wrapper import SICNavPolicyWrapper


def quaternion_to_yaw(z, w):
    return 2.0 * math.atan2(z, w)


class SICNavROS2BridgeNode(Node if HAVE_ROS2 else object):
    def __init__(self, policy_name='campc'):
        if HAVE_ROS2:
            super().__init__('sicnav_ros2_bridge_node')
            self.declare_parameter('policy', policy_name)
            self.declare_parameter('control_rate_hz', 10.0)
            self.declare_parameter('goal_x', 0.0)
            self.declare_parameter('goal_y', 5.0)

            pol = self.get_parameter('policy').get_parameter_value().string_value
            rate = self.get_parameter('control_rate_hz').get_parameter_value().double_value
            gx = self.get_parameter('goal_x').get_parameter_value().double_value
            gy = self.get_parameter('goal_y').get_parameter_value().double_value

            self.get_logger().info(f'Starting SICNav ROS 2 Bridge with policy: {pol}, rate: {rate}Hz, goal: ({gx}, {gy})')
        else:
            pol = policy_name
            rate = 10.0
            gx = 0.0
            gy = 5.0
            print(f'[SICNavBridge] Starting in Standalone mode with policy: {pol}')

        self.policy_wrapper = SICNavPolicyWrapper(policy_name=pol)

        # Robot State
        self.rx = 0.0
        self.ry = -5.0
        self.rtheta = math.pi / 2.0
        self.rvx = 0.0
        self.rvy = 0.0
        self.rw = 0.0

        # Goal State
        self.gx = gx
        self.gy = gy
        self.goal_reached = False

        # Crowd & Static Obstacles State
        self.humans = []
        self.static_obs = [
            (-2.5, 1.5, 0.4),  # Cylinder obstacle 1
            (2.5, -1.5, 0.4),  # Cylinder obstacle 2
        ]

        if HAVE_ROS2:
            # Subscribers
            self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
            self.crowd_sub = self.create_subscription(String, '/crowd/human_states', self.crowd_callback, 10)
            self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
            self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

            # Publishers
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.status_pub = self.create_publisher(String, '/sicnav/status', 10)

            # Control Timer
            timer_period = 1.0 / rate
            self.timer = self.create_timer(timer_period, self.control_loop)

    def odom_callback(self, msg: Odometry):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.rtheta = quaternion_to_yaw(qz, qw)

        self.rvx = msg.twist.twist.linear.x * math.cos(self.rtheta)
        self.rvy = msg.twist.twist.linear.x * math.sin(self.rtheta)
        self.rw = msg.twist.twist.angular.z

    def crowd_callback(self, msg: String):
        try:
            self.humans = json.loads(msg.data)
        except Exception:
            pass

    def goal_callback(self, msg: PoseStamped):
        self.gx = msg.pose.position.x
        self.gy = msg.pose.position.y
        self.goal_reached = False
        if HAVE_ROS2:
            self.get_logger().info(f'Updated Goal Position: ({self.gx:.2f}, {self.gy:.2f})')

    def scan_callback(self, msg: LaserScan):
        # Optional: extract dynamic obstacle points if needed
        pass

    def control_loop(self):
        dist_to_goal = math.hypot(self.gx - self.rx, self.gy - self.ry)

        if dist_to_goal < 0.25:
            if not self.goal_reached:
                self.goal_reached = True
                if HAVE_ROS2:
                    self.get_logger().info('Goal Reached! Stopping TurtleBot 4.')
            cmd = Twist() if HAVE_ROS2 else None
            if HAVE_ROS2:
                self.cmd_vel_pub.publish(cmd)
            self._publish_status(dist_to_goal, 0.0, 0.0)
            return

        # Execute SICNav Policy Planning
        robot_pose = (self.rx, self.ry, self.rtheta)
        robot_vel = (self.rvx, self.rvy, self.rw)
        goal = (self.gx, self.gy)

        v, w = self.policy_wrapper.plan_velocity(
            robot_pose, robot_vel, goal, self.humans, self.static_obs
        )

        if HAVE_ROS2:
            cmd = Twist()
            cmd.linear.x = float(v)
            cmd.angular.z = float(w)
            self.cmd_vel_pub.publish(cmd)

        self._publish_status(dist_to_goal, v, w)

    def _publish_status(self, dist, v, w):
        status_info = {
            "robot_pos": [round(self.rx, 2), round(self.ry, 2), round(self.rtheta, 2)],
            "dist_to_goal": round(dist, 2),
            "cmd_v": round(v, 2),
            "cmd_w": round(w, 2),
            "num_humans_tracked": len(self.humans),
            "goal_reached": self.goal_reached
        }
        if HAVE_ROS2:
            msg = String()
            msg.data = json.dumps(status_info)
            self.status_pub.publish(msg)


def main(args=None):
    if HAVE_ROS2:
        rclpy.init(args=args)
        node = SICNavROS2BridgeNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        node = SICNavROS2BridgeNode()
        print('[SICNavBridge] Looping test simulation...')
        for _ in range(20):
            node.control_loop()
            time.sleep(0.1)


if __name__ == '__main__':
    main()

