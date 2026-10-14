#!/usr/bin/env python3
"""
ROS 2 Launch file for Webots Simulation, TurtleBot 4 Controllers, and SICNav Bridge.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    world_path = os.path.join(pkg_share, '..', 'simulator_world', 'worlds', 'turtlebot4_crowd_world.wbt')
    world_path = os.path.abspath(world_path)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Webots) clock if true'
    )

    # TurtleBot 4 Webots Driver Node
    turtlebot4_driver_node = Node(
        package='ros2_simulator_interface',
        executable='turtlebot4_controller',
        name='turtlebot4_webots_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Crowd Supervisor Node
    crowd_supervisor_node = Node(
        package='ros2_simulator_interface',
        executable='crowd_supervisor',
        name='crowd_supervisor_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # SICNav ROS 2 Bridge Planner Node
    sicnav_bridge_node = Node(
        package='sicnav_ros2_interface',
        executable='sicnav_node',
        name='sicnav_planner_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        turtlebot4_driver_node,
        crowd_supervisor_node,
        sicnav_bridge_node
    ])

