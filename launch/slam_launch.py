#!/usr/bin/env python

# SPDX-FileCopyrightText: 2026 Dmytro Varich
# SPDX-License-Identifier: MIT

"""Launch Cartographer SLAM and optional RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_tesla_slam')
    launch_dir = os.path.join(package_dir, 'launch')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')
    launch_webots = LaunchConfiguration('launch_webots')

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        remappings=[
            ('scan_1', '/scan_front'),
            ('scan_2', '/scan_rear'),
            ('odom', '/odom'),
        ],
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        arguments=[
            '-configuration_directory',
            os.path.join(package_dir, 'config'),
            '-configuration_basename',
            'cartographer.lua'
        ],
    )

    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'resolution': 0.05},
            {'publish_period_sec': 1.0}
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        arguments=[
            '-d',
            os.path.join(package_dir, 'config', 'rviz_slam_config.rviz')
        ],
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world',
                default_value='tesla_world.wbt',
                description=(
                    'Choose one of the world files from '
                    '`/webots_ros2_tesla_slam/worlds` directory'
                )
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation time if true'
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='false',
                description='Launch RViz for SLAM if true'
            ),
            DeclareLaunchArgument(
                'launch_webots',
                default_value='true',
                description='Launch Webots Tesla with lane follower enabled if true'
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'tesla_webots_launch.py')
                ),
                launch_arguments=[
                    ('world', world),
                    ('use_sim_time', use_sim_time),
                    ('lane_follower', 'true'),
                    ('static_map_to_odom', 'false'),
                ],
                condition=IfCondition(launch_webots),
            ),
            cartographer,
            occupancy_grid,
            rviz,
        ]
    )
