# SPDX-FileCopyrightText: 2026 Dmytro Varich
# SPDX-License-Identifier: Apache-2.0

"""Launch Webots Tesla with Cartographer SLAM and optional RViz.

This launch file is used to build or inspect maps. It starts the Webots Tesla
driver, the vision lane follower, Cartographer, the occupancy grid publisher,
and optional RViz. Cartographer consumes both front and rear laser scans plus
vehicle odometry.
"""

import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node

from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_tesla_slam')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')
    launch_webots = LaunchConfiguration('launch_webots')
    launch_rviz_without_webots = PythonExpression([
        "'", launch_webots, "' != 'true' and '", use_rviz, "' == 'true'"
    ])

    # Webots can be disabled when another launch file already owns the
    # simulation and this file is only providing SLAM/RViz nodes.
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
    )

    # The map is built with both lidars, matching the merged scan used later
    # by AMCL localization.
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

    rviz_after_webots = Node(
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

    rviz_without_webots = Node(
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
        condition=IfCondition(launch_rviz_without_webots)
    )

    waiting_rviz = WaitForControllerConnection(
        target_driver=tesla_driver,
        nodes_to_start=[rviz_after_webots]
    )

    # RViz waits for the Webots controller when Webots is launched here, which
    # avoids starting visualization before simulation time and TF are available.
    webots_group = GroupAction(
        actions=[
            robot_state_publisher,
            webots,
            webots._supervisor,
            tesla_driver,
            lane_follower,
            waiting_rviz,
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
        ],
        condition=IfCondition(launch_webots),
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
            webots_group,
            cartographer,
            occupancy_grid,
            rviz_without_webots,
        ]
    )
