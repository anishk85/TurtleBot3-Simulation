#!/usr/bin/env python3
"""
Ultra-Aggressive Exploration Parameters for Complete Coverage
With Recovery Behaviors: Backtracking, New Path, Skip Area
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
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Main exploration node
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
            {'min_frontier_size': 0.05},
            {'min_distance': 0.01},
            {'potential_scale': 0.1},
            {'gain_scale': 10.0},
            {'orientation_scale': 0.0},
            {'planner_frequency': 0.1},
            {'progress_timeout': 300.0},
            {'transform_tolerance': 3.0},
            {'frontier_travel_point': 'closest'},
            {'allow_unknown': True},
            {'explore_clear_space': True},
        ]
    )

    # Recovery node (must be implemented in your package)
    recovery_node = Node(
        package='turtlebot3_autonomous_exploration',
        executable='exploration_recovery',
        name='exploration_recovery',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        explore_node,
        recovery_node
    ])