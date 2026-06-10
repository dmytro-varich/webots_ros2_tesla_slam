#!/usr/bin/env python

# SPDX-FileCopyrightText: 2026 Dmytro Varich
# SPDX-License-Identifier: MIT

"""Launch Navigation2 and optional RViz."""

import os

import launch
from ament_index_python.packages import (
    get_package_share_directory,
    get_packages_with_prefixes,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('rviz')
    launch_webots = LaunchConfiguration('launch_webots')
    launch_without_webots = PythonExpression([
        "'", launch_webots, "' != 'true'"
    ])

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

    rviz_after_webots = Node(
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
    )

    nav2_nodes_after_webots = []
    nav2_nodes_without_webots = []

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
                'yaml_filename': map_yaml,
                'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
                'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': nav_through_poses_bt_xml,
                'bt_navigator.ros__parameters.bt_search_directories': str([bt_dir, nav2_bt_dir]),
            },
            convert_types=True,
        )

        nav2_bringup_source = PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        )

        nav2_launch_arguments = [
            ('slam', 'False'),
            ('map', map_yaml),
            ('use_sim_time', use_sim_time),
            ('autostart', 'True'),
            ('use_composition', 'False'),
            ('use_respawn', 'True'),
            ('params_file', nav2_params),
        ]

        nav2_nodes_after_webots.append(
            IncludeLaunchDescription(
                nav2_bringup_source,
                launch_arguments=nav2_launch_arguments,
            )
        )
        nav2_nodes_without_webots.append(
            IncludeLaunchDescription(
                nav2_bringup_source,
                launch_arguments=nav2_launch_arguments,
            )
        )

    cmd_vel_to_ackermann_after_webots = Node(
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
    )

    cmd_vel_to_ackermann_without_webots = Node(
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
    )

    rviz_without_webots = Node(
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
    )

    waiting_navigation_nodes = WaitForControllerConnection(
        target_driver=tesla_driver,
        nodes_to_start=(
            nav2_nodes_after_webots +
            [cmd_vel_to_ackermann_after_webots, rviz_after_webots]
        )
    )

    webots_group = GroupAction(
        actions=[
            robot_state_publisher,
            webots,
            webots._supervisor,
            tesla_driver,
            waiting_navigation_nodes,
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

    navigation_without_webots_group = GroupAction(
        actions=(
            nav2_nodes_without_webots +
            [cmd_vel_to_ackermann_without_webots, rviz_without_webots]
        ),
        condition=IfCondition(launch_without_webots),
    )

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
        webots_group,
        navigation_without_webots_group,
    ]

    return LaunchDescription(actions)
