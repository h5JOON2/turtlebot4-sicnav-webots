#!/usr/bin/env python3
"""
TurtleBot 4 Webots Controller & ROS 2 Driver
- Subscribes to /cmd_vel (geometry_msgs/msg/Twist)
- Controls TurtleBot 4 differential drive motors in Webots
- Publishes /odom (nav_msgs/msg/Odometry) and TF transforms
- Publishes /scan (sensor_msgs/msg/LaserScan) from Webots Lidar
"""

import math
import sys
import os
import time

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from tf2_ros import TransformBroadcaster
    HAVE_ROS2 = True
except ImportError:
    HAVE_ROS2 = False

# Auto-detect Webots Python controller API
WEBOTS_HOME = os.environ.get('WEBOTS_HOME', '/Applications/Webots.app/Contents')
webots_py_path = os.path.join(WEBOTS_HOME, 'lib', 'controller', 'python')
if os.path.isdir(webots_py_path) and webots_py_path not in sys.path:
    sys.path.insert(0, webots_py_path)

if 'WEBOTS_CONTROLLER_URL' not in os.environ:
    os.environ['WEBOTS_CONTROLLER_URL'] = 'turtlebot4'

try:
    from controller import Robot
    HAVE_WEBOTS = True
except ImportError:
    HAVE_WEBOTS = False


# TurtleBot 4 Physical Parameters (Create 3 base)
WHEEL_RADIUS = 0.036     # [m]
WHEEL_BASE = 0.233       # [m]
MAX_WHEEL_SPEED = 20.0   # [rad/s]


class TurtleBot4WebotsNode(Node if HAVE_ROS2 else object):
    def __init__(self, webots_robot=None):
        if HAVE_ROS2:
            super().__init__('turtlebot4_webots_controller')
            self.get_logger().info('Initializing TurtleBot 4 Webots Controller...')
        else:
            print('[TurtleBot4] Initializing TurtleBot 4 Webots Controller (Standalone mode)...')

        self.robot = webots_robot
        self.time_step = 32
        if self.robot:
            self.time_step = int(self.robot.getBasicTimeStep())

        # Setup Devices
        self._setup_hardware()

        # State Variables
        self.x = 0.0
        self.y = -5.0
        self.theta = math.pi / 2.0  # Facing positive Y
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0

        self.last_left_pos = 0.0
        self.last_right_pos = 0.0
        self.first_step = True

        if HAVE_ROS2:
            # ROS 2 Subscribers & Publishers
            self.cmd_vel_sub = self.create_subscription(
                Twist, '/cmd_vel', self.cmd_vel_callback, 10
            )
            self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
            self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
            self.tf_broadcaster = TransformBroadcaster(self)

    def _setup_hardware(self):
        if not self.robot:
            return

        # Motors
        self.left_motor = self.robot.getDevice('left_wheel_motor')
        self.right_motor = self.robot.getDevice('right_wheel_motor')
        if self.left_motor and self.right_motor:
            self.left_motor.setPosition(float('inf'))
            self.right_motor.setPosition(float('inf'))
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)

        # Position Sensors (Encoders)
        self.left_sensor = self.robot.getDevice('left_wheel_sensor')
        self.right_sensor = self.robot.getDevice('right_wheel_sensor')
        if self.left_sensor and self.right_sensor:
            self.left_sensor.enable(self.time_step)
            self.right_sensor.enable(self.time_step)

        # Lidar Sensor
        self.lidar = self.robot.getDevice('rplidar')
        if self.lidar:
            self.lidar.enable(self.time_step)
            self.lidar.enablePointCloud()

        # Inertial Unit (IMU)
        self.imu = self.robot.getDevice('inertial_unit')
        if self.imu:
            self.imu.enable(self.time_step)

    def cmd_vel_callback(self, msg: Twist):
        self.target_linear_vel = msg.linear.x
        self.target_angular_vel = msg.angular.z

    def step(self):
        # Calculate Differential Drive wheel angular velocities
        v = self.target_linear_vel
        w = self.target_angular_vel

        # v_l = (v - w * L/2) / R,  v_r = (v + w * L/2) / R
        v_left = (v - (w * WHEEL_BASE / 2.0)) / WHEEL_RADIUS
        v_right = (v + (w * WHEEL_BASE / 2.0)) / WHEEL_RADIUS

        # Clamp to max speed
        v_left = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, v_left))
        v_right = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, v_right))

        if self.robot and self.left_motor and self.right_motor:
            self.left_motor.setVelocity(v_left)
            self.right_motor.setVelocity(v_right)

        # Update Odometry from Encoders
        self._update_odometry()

        # Publish Lidar Scan
        self._publish_lidar()

    def _update_odometry(self):
        dt = self.time_step / 1000.0
        now_time = self.get_clock().now() if HAVE_ROS2 else None

        if self.robot and self.left_sensor and self.right_sensor:
            curr_left = self.left_sensor.getValue()
            curr_right = self.right_sensor.getValue()

            if self.first_step:
                self.last_left_pos = curr_left
                self.last_right_pos = curr_right
                self.first_step = False
                return

            d_left = (curr_left - self.last_left_pos) * WHEEL_RADIUS
            d_right = (curr_right - self.last_right_pos) * WHEEL_RADIUS
            self.last_left_pos = curr_left
            self.last_right_pos = curr_right

            d_center = (d_left + d_right) / 2.0
            d_theta = (d_right - d_left) / WHEEL_BASE

            # If IMU is present, get heading from IMU
            if self.imu:
                rpy = self.imu.getRollPitchYaw()
                self.theta = rpy[2]  # Yaw
            else:
                self.theta += d_theta

            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)
        else:
            # Fallback dead reckoning simulation
            v = self.target_linear_vel
            w = self.target_angular_vel
            self.theta += w * dt
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt

        if HAVE_ROS2:
            # Publish Odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = now_time.to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'

            odom_msg.pose.pose.position.x = float(self.x)
            odom_msg.pose.pose.position.y = float(self.y)
            odom_msg.pose.pose.position.z = 0.0

            # Yaw to Quaternion (around Z axis)
            odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
            odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

            odom_msg.twist.twist.linear.x = float(self.target_linear_vel)
            odom_msg.twist.twist.angular.z = float(self.target_angular_vel)

            self.odom_pub.publish(odom_msg)

            # Broadcast TF (odom -> base_link)
            t = TransformStamped()
            t.header.stamp = now_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0
            t.transform.rotation = odom_msg.pose.pose.orientation

            self.tf_broadcaster.sendTransform(t)

    def _publish_lidar(self):
        if not HAVE_ROS2 or not self.robot or not self.lidar:
            return

        range_image = self.lidar.getRangeImage()
        if not range_image:
            return

        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'rplidar_link'

        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = (2.0 * math.pi) / len(range_image)
        scan_msg.time_increment = (self.time_step / 1000.0) / len(range_image)
        scan_msg.scan_time = self.time_step / 1000.0
        scan_msg.range_min = float(self.lidar.getMinRange())
        scan_msg.range_max = float(self.lidar.getMaxRange())

        scan_msg.ranges = [float(r) for r in range_image]
        self.scan_pub.publish(scan_msg)


def main(args=None):
    if HAVE_ROS2:
        rclpy.init(args=args)

    webots_robot = Robot() if HAVE_WEBOTS else None
    node = TurtleBot4WebotsNode(webots_robot)

    try:
        if webots_robot:
            time_step = int(webots_robot.getBasicTimeStep())
            while webots_robot.step(time_step) != -1:
                node.step()
                if HAVE_ROS2:
                    rclpy.spin_once(node, timeout_sec=0.001)
        else:
            print('[TurtleBot4] Running in mock/loop mode without Webots runtime.')
            while True:
                node.step()
                if HAVE_ROS2:
                    rclpy.spin_once(node, timeout_sec=0.03)
                time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        if HAVE_ROS2:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

