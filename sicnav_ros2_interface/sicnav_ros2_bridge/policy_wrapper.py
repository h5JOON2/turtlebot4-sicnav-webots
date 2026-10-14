#!/usr/bin/env python3
"""
Policy Wrapper for SICNav Algorithms
Wraps SICNav policies (CollisionAvoidMPC, DWA, ORCA) into a clean, uniform Python interface
for ROS 2 nodes. Provides robust fallback to DWA / potential fields if complex solver
backends (e.g. acados/casadi HSL) are not compiled on the target machine.
"""

import math
import os
import sys
import configparser
import numpy as np

# Ensure sicnav repository root is in sys.path
SICNAV_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'sicnav'))
if SICNAV_DIR not in sys.path:
    sys.path.insert(0, SICNAV_DIR)

HAVE_SICNAV = False
try:
    from crowd_sim_plus.envs.utils.state_plus import FullState, ObservableState, FullyObservableJointState
    from crowd_sim_plus.envs.utils.action import ActionRot
    from crowd_sim_plus.envs.utils.human_plus import Human
    HAVE_SICNAV = True
except ImportError:
    HAVE_SICNAV = False


class MockHumanState:
    def __init__(self, px, py, vx, vy, radius=0.3):
        self.px = px
        self.py = py
        self.vx = vx
        self.vy = vy
        self.radius = radius
        self.position = (px, py)
        self.velocity = (vx, vy)


class MockRobotState:
    def __init__(self, px, py, vx, vy, radius, heading, v_pref, gx, gy, omega=0.0):
        self.px = px
        self.py = py
        self.vx = vx
        self.vy = vy
        self.radius = radius
        self.heading = heading
        self.theta = heading
        self.v_pref = v_pref
        self.gx = gx
        self.gy = gy
        self.omega = omega
        self.position = (px, py)
        self.velocity = (vx, vy)
        self.goal_position = (gx, gy)


class MockJointState:
    def __init__(self, self_state, human_states, static_obs=None):
        self.self_state = self_state
        self.human_states = human_states
        self.static_obs = static_obs if static_obs is not None else []


class FallbackDWAPlanner:
    """Lightweight self-contained Dynamic Window Approach for social collision avoidance."""
    def __init__(self, max_speed=0.5, max_yaw_rate=1.5, max_accel=1.0, max_ang_accel=2.0, radius=0.25):
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.max_accel = max_accel
        self.max_ang_accel = max_ang_accel
        self.radius = radius
        self.dt = 0.1
        self.predict_time = 2.0

    def plan(self, x, y, theta, v, w, gx, gy, obstacles):
        # Calculate dynamic window
        vs = [0.0, self.max_speed, -self.max_yaw_rate, self.max_yaw_rate]
        vd = [v - self.max_accel * self.dt,
              v + self.max_accel * self.dt,
              w - self.max_ang_accel * self.dt,
              w + self.max_ang_accel * self.dt]
        dw = [max(vs[0], vd[0]), min(vs[1], vd[1]), max(vs[2], vd[2]), min(vs[3], vd[3])]

        best_u = [0.0, 0.0]
        min_cost = float('inf')

        # Sample velocity commands
        v_samples = np.linspace(dw[0], dw[1], 11)
        w_samples = np.linspace(dw[2], dw[3], 15)

        for vt in v_samples:
            for wt in w_samples:
                # Predict trajectory
                traj_x, traj_y, traj_th = x, y, theta
                dist_to_obs = float('inf')
                collision = False

                for t_step in np.arange(0, self.predict_time, self.dt):
                    traj_th += wt * self.dt
                    traj_x += vt * math.cos(traj_th) * self.dt
                    traj_y += vt * math.sin(traj_th) * self.dt

                    for ox, oy, orad in obstacles:
                        d = math.hypot(traj_x - ox, traj_y - oy) - (self.radius + orad)
                        if d < 0.05:
                            collision = True
                            break
                        if d < dist_to_obs:
                            dist_to_obs = d
                    if collision:
                        break

                if collision:
                    continue

                # Goal distance cost
                goal_cost = math.hypot(traj_x - gx, traj_y - gy)
                # Obstacle cost (prefer staying far from humans)
                obs_cost = 1.0 / (dist_to_obs + 1e-4) if dist_to_obs < 1.5 else 0.0
                # Speed reward (prefer moving forward)
                speed_cost = (self.max_speed - vt)

                total_cost = 1.2 * goal_cost + 1.5 * obs_cost + 0.3 * speed_cost
                if total_cost < min_cost:
                    min_cost = total_cost
                    best_u = [vt, wt]

        return best_u[0], best_u[1]


class SICNavPolicyWrapper:
    """Wrapper that manages SICNav policies (CAMPC / DWA / Fallback)."""

    def __init__(self, policy_name='campc', config_dir=None):
        self.policy_name = policy_name.lower()
        self.policy = None
        self.dt = 0.1
        self.max_speed = 0.45
        self.robot_radius = 0.22
        self.fallback_planner = FallbackDWAPlanner(
            max_speed=self.max_speed,
            max_yaw_rate=1.5,
            radius=self.robot_radius
        )

        self._load_policy(config_dir)

    def _load_policy(self, config_dir):
        if not HAVE_SICNAV:
            print("[SICNavWrapper] Notice: CrowdSimPlus/SICNav core not in python path; using internal social DWA planner.")
            return

        try:
            if self.policy_name in ['campc', 'sicnav']:
                from sicnav.policy.campc import CollisionAvoidMPC
                self.policy = CollisionAvoidMPC()
                print("[SICNavWrapper] Initialized SICNav Collision Avoidance MPC policy.")
            elif self.policy_name == 'dwa':
                from sicnav.policy.dwa import DynamicWindowApproach
                self.policy = DynamicWindowApproach()
                print("[SICNavWrapper] Initialized SICNav Dynamic Window Approach policy.")
            else:
                print(f"[SICNavWrapper] Unknown policy '{self.policy_name}'. Fallback to DWA planner.")
        except Exception as e:
            print(f"[SICNavWrapper] Could not instantiate {self.policy_name} ({e}). Using robust fallback planner.")
            self.policy = None

    def plan_velocity(self, robot_pose, robot_vel, goal, humans, static_obs=None):
        """
        Compute command velocities (v, w) based on current robot and crowd state.
        :param robot_pose: (px, py, theta)
        :param robot_vel: (vx, vy, omega)
        :param goal: (gx, gy)
        :param humans: list of dicts [{'px': .., 'py': .., 'vx': .., 'vy': .., 'radius': ..}, ...]
        :param static_obs: list of static obstacle segments or circles
        :return: (linear_v, angular_w)
        """
        rx, ry, rtheta = robot_pose
        rvx, rvy, rw = robot_vel
        gx, gy = goal

        # Check goal arrival
        dist_to_goal = math.hypot(gx - rx, gy - ry)
        if dist_to_goal < 0.25:
            return 0.0, 0.0

        # Try executing SICNav policy if loaded
        if self.policy is not None and HAVE_SICNAV:
            try:
                # Construct state compatible with CrowdSimPlus
                self_state = FullState(
                    px=rx, py=ry, vx=rvx, vy=rvy, radius=self.robot_radius,
                    heading=rtheta, v_pref=self.max_speed, gx=gx, gy=gy
                )
                self_state.omega = rw
                self_state.theta = rtheta
                self_state.goal_position = (gx, gy)

                human_objs = []
                for h in humans:
                    hum_st = ObservableState(
                        px=h.get('px', 0.0),
                        py=h.get('py', 0.0),
                        vx=h.get('vx', 0.0),
                        vy=h.get('vy', 0.0),
                        radius=h.get('radius', 0.3)
                    )
                    hum_st.position = (hum_st.px, hum_st.py)
                    hum_st.velocity = (hum_st.vx, hum_st.vy)
                    human_objs.append(hum_st)

                joint_state = MockJointState(self_state, human_objs, static_obs)
                action = self.policy.predict(joint_state)

                if hasattr(action, 'v') and hasattr(action, 'r'):
                    v = float(action.v)
                    w = float(action.r) / self.dt
                    return v, w
            except Exception as e:
                # If solver triggers any runtime issues, gracefully fall back to DWA
                pass

        # Robust Fallback DWA Social Planner
        obs_list = []
        for h in humans:
            obs_list.append((h.get('px', 0.0), h.get('py', 0.0), h.get('radius', 0.3)))
        if static_obs:
            for so in static_obs:
                if len(so) == 3:
                    obs_list.append((so[0], so[1], so[2]))

        v, w = self.fallback_planner.plan(rx, ry, rtheta, math.hypot(rvx, rvy), rw, gx, gy, obs_list)
        return float(v), float(w)

