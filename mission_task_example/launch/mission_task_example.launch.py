#!/usr/bin/env python3
"""
Launch file for Mission-Task Example

This launch file starts the mission FSM node that coordinates
the execution of three BT tasks.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('mission_task_example')
    
    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    # Mission FSM Node
    mission_fsm_node = Node(
        package='mission_task_example',
        executable='mission_fsm_example',
        name='mission_fsm_node',
        output='screen',
        parameters=[],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    return LaunchDescription([
        log_level_arg,
        LogInfo(msg=['Starting Mission-Task Integration Example']),
        LogInfo(msg=['Package directory: ', pkg_dir]),
        mission_fsm_node
    ])
