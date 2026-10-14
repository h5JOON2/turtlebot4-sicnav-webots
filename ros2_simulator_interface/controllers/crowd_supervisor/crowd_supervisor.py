#!/usr/bin/env python3
"""
Webots Crowd Supervisor Controller & ROS 2 Publisher
- Controls pedestrian movement in the Webots world (patrol/crossing behaviors)
- Queries current positions and velocities of all humans in the simulation
- Publishes /crowd/human_states (std_msgs/msg/String JSON & geometry_msgs/msg/PoseArray)
  for SICNav to predict and plan collision-free paths.
"""

import math
import sys
import os
import json
import time

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseArray, Pose
    HAVE_ROS2 = True
except ImportError:
    HAVE_ROS2 = False

# Auto-detect Webots Python controller API
WEBOTS_HOME = os.environ.get('WEBOTS_HOME', '/Applications/Webots.app/Contents')
webots_py_path = os.path.join(WEBOTS_HOME, 'lib', 'controller', 'python')
if os.path.isdir(webots_py_path) and webots_py_path not in sys.path:
    sys.path.insert(0, webots_py_path)

if 'WEBOTS_CONTROLLER_URL' not in os.environ:
    os.environ['WEBOTS_CONTROLLER_URL'] = 'crowd_supervisor'

try:
    from controller import Supervisor
    HAVE_WEBOTS = True
except ImportError:
    HAVE_WEBOTS = False


class PedestrianAgent:
    def __init__(self, name, node, waypoints, speed=0.8):
        self.name = name
        self.node = node
        self.waypoints = waypoints
        self.current_wp_idx = 0
        self.speed = speed
        self.px = waypoints[0][0]
        self.py = waypoints[0][1]
        self.vx = 0.0
        self.vy = 0.0
        self.radius = 0.3

        if self.node:
            self.trans_field = self.node.getField("translation")

    def step(self, dt):
        target_x, target_y = self.waypoints[self.current_wp_idx]
        dx = target_x - self.px
        dy = target_y - self.py
        dist = math.hypot(dx, dy)

        if dist < 0.2:
            self.current_wp_idx = (self.current_wp_idx + 1) % len(self.waypoints)
            target_x, target_y = self.waypoints[self.current_wp_idx]
            dx = target_x - self.px
            dy = target_y - self.py
            dist = math.hypot(dx, dy)

        if dist > 1e-4:
            self.vx = (dx / dist) * self.speed
            self.vy = (dy / dist) * self.speed
        else:
            self.vx = 0.0
            self.vy = 0.0

        self.px += self.vx * dt
        self.py += self.vy * dt

        if self.node and self.trans_field:
            self.trans_field.setSFVec3f([float(self.px), float(self.py), 0.9])


class CrowdSupervisorNode(Node if HAVE_ROS2 else object):
    def __init__(self, supervisor=None):
        if HAVE_ROS2:
            super().__init__('crowd_supervisor_node')
            self.get_logger().info('Initializing Webots Crowd Supervisor Node...')
        else:
            print('[CrowdSupervisor] Initializing Webots Crowd Supervisor Node (Standalone mode)...')

        self.supervisor = supervisor
        self.time_step = 32
        if self.supervisor:
            self.time_step = int(self.supervisor.getBasicTimeStep())

        # Setup Pedestrians
        self.humans = []
        self._init_pedestrians()

        if HAVE_ROS2:
            # Publisher for detailed JSON human states (compatible with SICNav)
            self.json_pub = self.create_publisher(String, '/crowd/human_states', 10)
            # Publisher for standard ROS 2 PoseArray (for RViz visualization)
            self.pose_pub = self.create_publisher(PoseArray, '/crowd/human_poses', 10)

    def _init_pedestrians(self):
        human_configs = [
            ("HUMAN_0", [(-2.0, -1.0), (2.0, -1.0)], 0.6),
            ("HUMAN_1", [(1.5, 0.0), (-1.5, 0.0)], 0.7),
            ("HUMAN_2", [(-1.0, 2.5), (1.0, 2.5)], 0.5),
            ("HUMAN_3", [(2.0, 2.0), (2.0, -2.0)], 0.8),
        ]

        for def_name, wps, spd in human_configs:
            node = None
            if self.supervisor:
                node = self.supervisor.getFromDef(def_name)
            agent = PedestrianAgent(def_name, node, wps, spd)
            self.humans.append(agent)

    def step(self):
        dt = self.time_step / 1000.0

        states_data = []
        for h in self.humans:
            h.step(dt)
            states_data.append({
                "id": h.name,
                "px": float(h.px),
                "py": float(h.py),
                "vx": float(h.vx),
                "vy": float(h.vy),
                "radius": float(h.radius)
            })

        if HAVE_ROS2:
            # 1. Publish JSON
            json_msg = String()
            json_msg.data = json.dumps(states_data)
            self.json_pub.publish(json_msg)

            # 2. Publish PoseArray for RViz
            pose_array = PoseArray()
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = "odom"

            for h in self.humans:
                p = Pose()
                p.position.x = float(h.px)
                p.position.y = float(h.py)
                p.position.z = 0.0
                yaw = math.atan2(h.vy, h.vx) if math.hypot(h.vx, h.vy) > 1e-3 else 0.0
                p.orientation.z = math.sin(yaw / 2.0)
                p.orientation.w = math.cos(yaw / 2.0)
                pose_array.poses.append(p)

            self.pose_pub.publish(pose_array)


def main(args=None):
    if HAVE_ROS2:
        rclpy.init(args=args)

    supervisor = Supervisor() if HAVE_WEBOTS else None
    node = CrowdSupervisorNode(supervisor)

    try:
        if supervisor:
            time_step = int(supervisor.getBasicTimeStep())
            while supervisor.step(time_step) != -1:
                node.step()
                if HAVE_ROS2:
                    rclpy.spin_once(node, timeout_sec=0.001)
        else:
            print('[CrowdSupervisor] Running in mock/loop mode without Webots runtime.')
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

