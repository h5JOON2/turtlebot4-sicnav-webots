#!/usr/bin/env python3
"""
Interactive / Animated Simulation Runner for TurtleBot 4 & SICNav
Visualizes:
- Arena walls & Static obstacles (cylinder pillars)
- TurtleBot 4 (differential drive robot, orientation, path history)
- Pedestrians (dynamic agents moving across the path)
- Real-time Collision Avoidance decisions (cmd_vel: v, w)
Generates an animated GIF and plot summary.
"""

import os
import sys
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Render to animated GIF / images headless-safe
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(PROJECT_DIR, 'sicnav_ros2_interface'))

from sicnav_ros2_bridge.policy_wrapper import SICNavPolicyWrapper


def run_animated_simulation(output_gif='turtlebot4_sicnav_simulation.gif'):
    print(f"[Sim] Setting up simulation environment...")
    wrapper = SICNavPolicyWrapper(policy_name='campc')

    # Simulation Setup
    dt = 0.1
    max_steps = 120
    robot_radius = 0.22

    # Robot Initial State: (0, -5), heading North (pi/2)
    robot_pose = [0.0, -5.0, math.pi / 2.0]
    robot_vel = [0.0, 0.0, 0.0]
    goal = (0.0, 5.0)

    # Static Obstacles (matches Webots world: (x, y, radius))
    static_obs = [
        (-2.5, 1.5, 0.4),
        (2.5, -1.5, 0.4),
    ]

    # Pedestrians (matches Webots crowd supervisor)
    humans = [
        {"id": "HUMAN_0", "px": -2.0, "py": -1.0, "vx": 0.5, "vy": 0.0, "radius": 0.3, "wps": [(-2.0, -1.0), (2.0, -1.0)], "wp_idx": 1},
        {"id": "HUMAN_1", "px": 1.5, "py": 0.0, "vx": -0.6, "vy": 0.0, "radius": 0.3, "wps": [(1.5, 0.0), (-1.5, 0.0)], "wp_idx": 1},
        {"id": "HUMAN_2", "px": -1.0, "py": 2.5, "vx": 0.4, "vy": 0.0, "radius": 0.3, "wps": [(-1.0, 2.5), (1.0, 2.5)], "wp_idx": 1},
        {"id": "HUMAN_3", "px": 2.0, "py": 2.0, "vx": 0.0, "vy": -0.7, "radius": 0.3, "wps": [(2.0, 2.0), (2.0, -2.0)], "wp_idx": 1},
    ]

    history_robot_x = []
    history_robot_y = []
    history_humans_x = [[] for _ in humans]
    history_humans_y = [[] for _ in humans]
    sim_data = []

    print(f"[Sim] Simulating {max_steps} steps of SICNav crowd navigation...")
    for step in range(max_steps):
        rx, ry, rth = robot_pose
        history_robot_x.append(rx)
        history_robot_y.append(ry)

        for i, h in enumerate(humans):
            history_humans_x[i].append(h["px"])
            history_humans_y[i].append(h["py"])

        # Check goal arrival
        dist_to_goal = math.hypot(goal[0] - rx, goal[1] - ry)
        if dist_to_goal < 0.25:
            print(f" Goal reached at step {step}! (distance: {dist_to_goal:.2f}m)")
            sim_data.append((list(robot_pose), [dict(h) for h in humans], 0.0, 0.0, dist_to_goal))
            break

        # Plan velocity using SICNav
        v, w = wrapper.plan_velocity(
            (rx, ry, rth), tuple(robot_vel), goal, humans, static_obs
        )

        sim_data.append((list(robot_pose), [dict(h) for h in humans], v, w, dist_to_goal))

        # Update Robot State (Differential Drive Kinematics)
        rth += w * dt
        rx += v * math.cos(rth) * dt
        ry += v * math.sin(rth) * dt
        robot_pose = [rx, ry, rth]
        robot_vel = [v * math.cos(rth), v * math.sin(rth), w]

        # Update Human Positions
        for h in humans:
            target_x, target_y = h["wps"][h["wp_idx"]]
            dx = target_x - h["px"]
            dy = target_y - h["py"]
            d = math.hypot(dx, dy)
            if d < 0.2:
                h["wp_idx"] = 1 - h["wp_idx"]
                target_x, target_y = h["wps"][h["wp_idx"]]
                dx = target_x - h["px"]
                dy = target_y - h["py"]
                d = math.hypot(dx, dy)
            spd = math.hypot(h["vx"], h["vy"]) if math.hypot(h["vx"], h["vy"]) > 1e-3 else 0.6
            h["vx"] = (dx / (d + 1e-4)) * spd
            h["vy"] = (dy / (d + 1e-4)) * spd
            h["px"] += h["vx"] * dt
            h["py"] += h["vy"] * dt

    print(f"[Sim] Generating high-resolution animation...")
    fig, ax = plt.subplots(figsize=(7, 7), dpi=100)

    def update(frame):
        ax.clear()
        pose, h_list, v, w, dist = sim_data[frame]

        # Draw Arena Walls
        ax.plot([-7, 7, 7, -7, -7], [-7, -7, 7, 7, -7], 'k-', lw=3, label='Arena Wall')
        ax.set_xlim(-7.5, 7.5)
        ax.set_ylim(-7.5, 7.5)
        ax.set_aspect('equal')
        ax.grid(True, linestyle='--', alpha=0.5)

        # Draw Goal
        goal_circle = patches.Circle(goal, 0.4, color='green', alpha=0.3, label='Goal')
        ax.add_patch(goal_circle)
        ax.plot(goal[0], goal[1], 'g*', markersize=12)

        # Draw Static Obstacles
        for ox, oy, orad in static_obs:
            obs_patch = patches.Circle((ox, oy), orad, color='#4A5568', alpha=0.8)
            ax.add_patch(obs_patch)

        # Draw Robot Path Trajectory
        ax.plot(history_robot_x[:frame+1], history_robot_y[:frame+1], 'b--', lw=1.5, alpha=0.6, label='TurtleBot 4 Path')

        # Draw Robot Body (Create 3 + Tower)
        rx, ry, rth = pose
        robot_body = patches.Circle((rx, ry), robot_radius, color='#2B6CB0', ec='black', lw=2, zorder=5)
        ax.add_patch(robot_body)
        # Heading indicator
        head_len = 0.35
        ax.arrow(rx, ry, head_len * math.cos(rth), head_len * math.sin(rth),
                 head_width=0.15, head_length=0.1, fc='red', ec='red', zorder=6)

        # Draw Humans
        human_colors = ['#DD6B20', '#319795', '#D53F8C', '#805AD5']
        for idx, h in enumerate(h_list):
            h_circle = patches.Circle((h["px"], h["py"]), h["radius"], color=human_colors[idx % len(human_colors)], ec='black', lw=1.5, zorder=5)
            ax.add_patch(h_circle)
            ax.text(h["px"], h["py"] + 0.4, h["id"], ha='center', fontsize=8, fontweight='bold', color='#2D3748')
            # Velocity arrow
            ax.arrow(h["px"], h["py"], h["vx"] * 0.4, h["vy"] * 0.4,
                     head_width=0.1, head_length=0.08, fc=human_colors[idx % len(human_colors)], ec='black', lw=0.5)

        ax.set_title(f"TurtleBot 4 Crowd Navigation (SICNav)\nStep: {frame:03d} | v: {v:+.2f} m/s, w: {w:+.2f} rad/s | Dist: {dist:.2f}m", fontsize=11)
        ax.legend(loc='lower right', fontsize=8)

    anim = FuncAnimation(fig, update, frames=len(sim_data), interval=80)
    out_path = os.path.join(PROJECT_DIR, output_gif)
    anim.save(out_path, writer='pillow', fps=12)
    plt.close()
    print(f" Animation saved to: {out_path}")

    # Also save static summary figure
    fig_summary, ax_sum = plt.subplots(figsize=(8, 8), dpi=120)
    ax_sum.plot([-7, 7, 7, -7, -7], [-7, -7, 7, 7, -7], 'k-', lw=2.5, label='Walls')
    for ox, oy, orad in static_obs:
        ax_sum.add_patch(patches.Circle((ox, oy), orad, color='#4A5568', alpha=0.8, label='Obstacle' if ox == static_obs[0][0] else ""))
    ax_sum.add_patch(patches.Circle(goal, 0.4, color='green', alpha=0.3, label='Goal Area'))
    ax_sum.plot(history_robot_x, history_robot_y, 'b-', lw=2.5, label='TurtleBot 4 Trajectory')
    for i in range(len(humans)):
        ax_sum.plot(history_humans_x[i], history_humans_y[i], ':', lw=1.5, label=f'Pedestrian {i}')
    ax_sum.plot(robot_pose[0], robot_pose[1], 'bo', markersize=8, label='Final Position')
    ax_sum.plot(0.0, -5.0, 'go', markersize=8, label='Start (0, -5)')
    ax_sum.plot(goal[0], goal[1], 'g*', markersize=14, label='Goal (0, 5)')
    ax_sum.set_xlim(-7.5, 7.5)
    ax_sum.set_ylim(-7.5, 7.5)
    ax_sum.set_aspect('equal')
    ax_sum.grid(True, linestyle='--', alpha=0.5)
    ax_sum.legend(loc='lower right')
    ax_sum.set_title("TurtleBot 4 SICNav Full Trajectory Summary", fontsize=13, fontweight='bold')
    summary_path = os.path.join(PROJECT_DIR, 'simulation_trajectory_summary.png')
    fig_summary.savefig(summary_path)
    plt.close()
    print(f" Trajectory summary plot saved to: {summary_path}")


if __name__ == '__main__':
    run_animated_simulation()

