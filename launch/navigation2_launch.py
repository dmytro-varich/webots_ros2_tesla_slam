#!/usr/bin/env python

# SPDX-FileCopyrightText: 2026 Dmytro Varich
# SPDX-License-Identifier: MIT

"""Launch Navigation2 and optional RViz."""

import os

from ament_index_python.packages import (
    get_package_share_directory,
    get_packages_with_prefixes,
)
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
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('rviz')
    launch_webots = LaunchConfiguration('launch_webots')

    actions = [
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
            'map',
            default_value=os.path.join(package_dir, 'maps', 'city_map.yaml'),
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz for Navigation2 if true'
        ),
        DeclareLaunchArgument(
            'launch_webots',
            default_value='true',
            description='Launch Webots Tesla with lane follower disabled if true'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'tesla_webots_launch.py')
            ),
            launch_arguments=[
                ('world', world),
                ('use_sim_time', use_sim_time),
                ('lane_follower', 'false'),
                ('static_map_to_odom', 'false'),
            ],
            condition=IfCondition(launch_webots),
        ),
    ]

    if 'nav2_bringup' in get_packages_with_prefixes():
        from nav2_common.launch import RewrittenYaml

        bt_dir = os.path.join(package_dir, 'behavior_trees')
        nav_to_pose_bt_xml = os.path.join(
            bt_dir, 'navigate_to_pose_w_replanning_and_recovery.xml'
        )
        nav_through_poses_bt_xml = os.path.join(
            bt_dir, 'navigate_through_poses_w_replanning_and_recovery.xml'
        )
        nav2_bt_dir = os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees',
        )

        nav2_params = RewrittenYaml(
            source_file=os.path.join(package_dir, 'config', 'nav2_params.yaml'),
            root_key='',
            param_rewrites={
                'map_server.ros__parameters.yaml_filename': map_yaml,
                'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
                'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': nav_through_poses_bt_xml,
                'bt_navigator.ros__parameters.bt_search_directories': str([bt_dir, nav2_bt_dir]),
            },
            convert_types=True,
        )

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'bringup_launch.py'
                    )
                ),
                launch_arguments=[
                    ('slam', 'False'),
                    ('map', map_yaml),
                    ('use_sim_time', use_sim_time),
                    ('autostart', 'True'),
                    ('use_composition', 'False'),
                    ('use_respawn', 'True'),
                    ('params_file', nav2_params),
                ],
            )
        )

    actions.extend(
        [
            Node(
                package='webots_ros2_tesla_slam',
                executable='cmd_vel_to_ackermann',
                name='cmd_vel_to_ackermann',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'wheelbase': 2.94,
                    'input_topic': '/cmd_vel',
                    'output_topic': '/cmd_ackermann'
                }],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_nav',
                arguments=[
                    '-d',
                    os.path.join(package_dir, 'config', 'rviz_nav_config.rviz')
                ],
                parameters=[
                    {'use_sim_time': use_sim_time}
                ],
                output='screen',
                condition=IfCondition(use_rviz)
            ),
        ]
    )

    return LaunchDescription(actions)
