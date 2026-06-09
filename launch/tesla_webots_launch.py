#!/usr/bin/env python

# SPDX-FileCopyrightText: 1996-2023 Cyberbotics Ltd.
# SPDX-FileCopyrightText: 2026 Dmytro Varich
# SPDX-License-Identifier: Apache-2.0

"""Launch only the Webots Tesla simulation and driver."""

import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node

from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_tesla_slam')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_lane_follower = LaunchConfiguration('lane_follower')
    publish_static_map_to_odom = LaunchConfiguration('static_map_to_odom')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    robot_description_path = os.path.join(
        package_dir, 'resource', 'tesla_webots.urdf'
    )
    with open(robot_description_path, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ]
    )

    tesla_driver = WebotsController(
        robot_name='vehicle',
        parameters=[
            {
                'robot_description': robot_description_path,
                'use_sim_time': use_sim_time
            }
        ],
        respawn=True
    )

    lane_follower = Node(
        package='webots_ros2_tesla_slam',
        executable='lane_follower',
        condition=IfCondition(launch_lane_follower),
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--yaw', '0',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(publish_static_map_to_odom)
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
                'lane_follower',
                default_value='true',
                description='Launch lane follower if true'
            ),
            DeclareLaunchArgument(
                'static_map_to_odom',
                default_value='false',
                description='Publish static map to odom transform if true'
            ),
            robot_state_publisher,
            webots,
            webots._supervisor,
            tesla_driver,
            lane_follower,
            map_to_odom,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[
                        launch.actions.EmitEvent(
                            event=launch.events.Shutdown()
                        )
                    ],
                )
            ),
        ]
    )
