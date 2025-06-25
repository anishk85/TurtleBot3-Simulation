#!/usr/bin/env python3
"""
Ultra-Aggressive Exploration Parameters for Complete Coverage
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Get launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # ULTRA-AGGRESSIVE exploration parameters
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_base_frame': 'base_link'},
            {'costmap_topic': 'global_costmap/costmap'},
            {'costmap_updates_topic': 'global_costmap/costmap_updates'},
            {'visualize': True},
            
            # CRITICAL: Make frontier detection very sensitive
            {'min_frontier_size': 0.05},        # Detect tiny frontiers
            {'min_distance': 0.01},             # Go very close to frontiers
            
            # Frontier evaluation - favor exploration over safety
            {'potential_scale': 0.1},           # Lower obstacle avoidance
            {'gain_scale': 10.0},               # Heavily favor unexplored areas
            {'orientation_scale': 0.0},         # Don't care about orientation
            
            # Timing - be more patient
            {'planner_frequency': 0.1},         # Plan less frequently
            {'progress_timeout': 300.0},        # 5 minute timeout
            {'transform_tolerance': 3.0},       # Very tolerant of delays
            
            # Navigation behavior
            {'frontier_travel_point': 'closest'}, # Go to closest point
            {'allow_unknown': True},            # Allow navigation through unknown
            {'explore_clear_space': True},      # Explore open areas too
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        explore_node
    ])