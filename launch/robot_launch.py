#!/usr/bin/env python

# SPDX-FileCopyrightText: 1996-2023 Cyberbotics Ltd.
# SPDX-FileCopyrightText: 2026 Dmytro Varich
# SPDX-License-Identifier: Apache-2.0

"""Launch Webots Tesla driver with SLAM and NAV2."""

import os

import launch
from ament_index_python.packages import (
    get_package_share_directory,
    get_packages_with_prefixes,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node

from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_tesla_slam')
    world = LaunchConfiguration('world')
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam = LaunchConfiguration('slam', default=False)
    use_rviz_nav2 = LaunchConfiguration('rviz_nav', default=False)
    use_rviz_slam = LaunchConfiguration('rviz_slam', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    map_yaml = LaunchConfiguration(
        'map',
        default=os.path.join(package_dir, 'maps', 'city_map.yaml')
    )

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    use_nav_without_slam = PythonExpression([
        "'", use_nav, "' == 'true' and '", use_slam, "' != 'true'"
    ])
    use_slam_without_nav = PythonExpression([
        "'", use_nav, "' != 'true' and '", use_slam, "' == 'true'"
    ])


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
        condition=UnlessCondition(use_nav),
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
        condition=UnlessCondition(
            PythonExpression(["'", use_slam, "' == 'true' or '", use_nav, "' == 'true'"])
        )
    )

    # Cartographer SLAM
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
            '-configuration_directory', os.path.join(
                get_package_share_directory('webots_ros2_tesla_slam'),
                'config'
            ),
            '-configuration_basename', 'cartographer.lua'
        ],
        condition=launch.conditions.IfCondition(use_slam_without_nav)
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
        condition=launch.conditions.IfCondition(use_slam_without_nav)
    )

    navigation_nodes = [
        cartographer,
        occupancy_grid,
    ]

    # Navigation
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

        # Our Nav2 params file needs absolute paths for map and BT XML.
        # YAML itself does not expand $(find-pkg-share ...) reliably, so we
        # rewrite these keys at launch time.
        nav2_params = RewrittenYaml(
            source_file=os.path.join(package_dir, 'config', 'nav2_params.yaml'),
            root_key='',
            param_rewrites={
                'map_server.ros__parameters.yaml_filename': map_yaml,
                'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
                'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': nav_through_poses_bt_xml,
                # bt_search_directories is a list param; pass it as a YAML list string.
                'bt_navigator.ros__parameters.bt_search_directories': str([bt_dir, nav2_bt_dir]),
            },
            convert_types=True,
        )
        nav2_launch = IncludeLaunchDescription(
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
                (
                    'params_file',
                    nav2_params
                ),
            ],
            condition=launch.conditions.IfCondition(use_nav_without_slam)
        )
        navigation_nodes.append(nav2_launch)

    # CmdVel to Ackermann converter for Navigation2
    ackermann_converter_node = Node(
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
        condition=launch.conditions.IfCondition(use_nav_without_slam)
    )
    navigation_nodes.append(ackermann_converter_node)

    # RViz SLAM
    rviz_slam_config_path = os.path.join(package_dir, 'config', 'rviz_slam_config.rviz')
    rviz_slam_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_slam_config_path],
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        condition=launch.conditions.IfCondition(use_rviz_slam)
    )

    # RViz Nav2
    rviz_nav_config_path = os.path.join(package_dir, 'config', 'rviz_nav_config.rviz')
    rviz_nav_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_nav_config_path],
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        condition=launch.conditions.IfCondition(use_rviz_nav2)
    )

    # Wait for the simulation to be ready to start navigation nodes
    waiting_nodes = WaitForControllerConnection(
        target_driver=tesla_driver,
        nodes_to_start=navigation_nodes + [rviz_slam_node, rviz_nav_node]
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
            robot_state_publisher,
            webots,
            webots._supervisor,
            tesla_driver,
            lane_follower,
            map_to_odom,
            waiting_nodes,
            # This action will kill all nodes once the Webots simulation
            # has exited
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
