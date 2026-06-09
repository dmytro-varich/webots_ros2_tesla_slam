#!/usr/bin/env python

# SPDX-FileCopyrightText: 1996-2023 Cyberbotics Ltd.
# SPDX-FileCopyrightText: 2026 Dmytro Varich
# SPDX-License-Identifier: Apache-2.0

"""Compatibility launch file for Webots Tesla, SLAM, and Nav2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_tesla_slam')
    launch_dir = os.path.join(package_dir, 'launch')

    world = LaunchConfiguration('world')
    use_nav = LaunchConfiguration('nav')
    use_slam = LaunchConfiguration('slam')
    use_rviz_nav = LaunchConfiguration('rviz_nav')
    use_rviz_slam = LaunchConfiguration('rviz_slam')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')

    use_nav_without_slam = PythonExpression([
        "'", use_nav, "' == 'true' and '", use_slam, "' != 'true'"
    ])
    use_slam_without_nav = PythonExpression([
        "'", use_nav, "' != 'true' and '", use_slam, "' == 'true'"
    ])
    use_lane_follower = PythonExpression([
        "'true' if '", use_nav, "' != 'true' else 'false'"
    ])
    publish_static_map_to_odom = PythonExpression([
        "'true' if '", use_slam, "' != 'true' and '", use_nav,
        "' != 'true' else 'false'"
    ])

    tesla_webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'tesla_webots_launch.py')
        ),
        launch_arguments=[
            ('world', world),
            ('use_sim_time', use_sim_time),
            ('lane_follower', use_lane_follower),
            ('static_map_to_odom', publish_static_map_to_odom),
        ],
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'slam_launch.py')
        ),
        launch_arguments=[
            ('world', world),
            ('use_sim_time', use_sim_time),
            ('rviz', use_rviz_slam),
            ('launch_webots', 'false'),
        ],
        condition=IfCondition(use_slam_without_nav),
    )

    navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'navigation2_launch.py')
        ),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('map', map_yaml),
            ('rviz', use_rviz_nav),
            ('launch_webots', 'false'),
        ],
        condition=IfCondition(use_nav_without_slam),
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
                'nav',
                default_value='false',
                description='Launch Navigation2 if true'
            ),
            DeclareLaunchArgument(
                'slam',
                default_value='false',
                description='Launch Cartographer SLAM if true'
            ),
            DeclareLaunchArgument(
                'rviz_nav',
                default_value='false',
                description='Launch RViz for Navigation2 if true'
            ),
            DeclareLaunchArgument(
                'rviz_slam',
                default_value='false',
                description='Launch RViz for SLAM if true'
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation time if true'
            ),
            DeclareLaunchArgument(
                'map',
                default_value=os.path.join(package_dir, 'maps', 'city_map.yaml'),
                description='Full path to map yaml file'
            ),
            tesla_webots_launch,
            slam_launch,
            navigation2_launch,
        ]
    )
