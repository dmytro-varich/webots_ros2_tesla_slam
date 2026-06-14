# SPDX-FileCopyrightText: 2026 Dmytro Varich
# SPDX-License-Identifier: Apache-2.0

"""Launch Webots Tesla with Nav2 and selectable localization.

This launch file starts the Tesla Webots driver, robot_state_publisher, Nav2,
the Twist-to-Ackermann bridge, and optional RViz. The localization mode is
selected with the `localization` argument:
    - gps: GPS, IMU, wheel odometry, navsat_transform, and dual EKF
    - amcl: map_server, AMCL, and merged front/rear laser scan
    - odom: map_server with a static map -> odom transform for debugging/demo
"""

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
    SetEnvironmentVariable,
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
    localization = LaunchConfiguration('localization')
    launch_without_webots = PythonExpression([
        "'", launch_webots, "' != 'true'"
    ])
    use_amcl = PythonExpression([
        "'", localization, "' == 'amcl'"
    ])
    use_odom = PythonExpression([
        "'", localization, "' == 'odom'"
    ])
    gps_enabled = PythonExpression([
        "'", localization, "' == 'gps'"
    ])
    driver_odom_topic = PythonExpression([
        "'/wheel/odom' if '", localization, "' == 'gps' else 'odom'"
    ])

    # Webots is still launched first; Nav2 starts only after the external
    # vehicle controller connects to the simulation.
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
                'use_sim_time': use_sim_time,
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
    gps_nodes_after_webots = []
    gps_nodes_without_webots = []

    if 'nav2_bringup' in get_packages_with_prefixes():
        from nav2_common.launch import RewrittenYaml

        # RewrittenYaml lets the same nav2_params.yaml serve all localization
        # modes while replacing runtime-only values such as the map path and
        # behavior tree file paths.
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

        common_param_rewrites = {
            'yaml_filename': map_yaml,
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
            'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': nav_through_poses_bt_xml,
            'bt_navigator.ros__parameters.bt_search_directories': str([bt_dir, nav2_bt_dir]),
        }
        amcl_param_rewrites = {
            **common_param_rewrites,
            # AMCL expects one LaserScan topic. The merger combines front and
            # rear Webots lidars into /scan in the base_link frame.
            'amcl.ros__parameters.scan_topic': '/scan',
            'amcl.ros__parameters.tf_broadcast': 'true',
            # AMCL mode keeps the planner forward-only while AMCL tuning is
            # separated from the GPS-specific reversing setup.
            'planner_server.ros__parameters.GridBased.motion_model_for_search': 'DUBIN',
            'planner_server.ros__parameters.GridBased.reverse_penalty': '2.0',
            'controller_server.ros__parameters.path_handler.enforce_path_inversion': 'false',
            'controller_server.ros__parameters.FollowPath.allow_reversing': 'false',
        }
        odom_param_rewrites = {
            **common_param_rewrites,
            # Odom mode is a local-odometry debug/demo mode, so keep the same
            # forward-only behavior as AMCL and avoid reverse path inversions.
            'planner_server.ros__parameters.GridBased.motion_model_for_search': 'DUBIN',
            'planner_server.ros__parameters.GridBased.reverse_penalty': '2.0',
            'controller_server.ros__parameters.path_handler.enforce_path_inversion': 'false',
            'controller_server.ros__parameters.FollowPath.allow_reversing': 'false',
        }

        amcl_nav2_params = RewrittenYaml(
            source_file=os.path.join(package_dir, 'config', 'nav2_params.yaml'),
            root_key='',
            param_rewrites=amcl_param_rewrites,
            convert_types=True,
        )

        odom_nav2_params = RewrittenYaml(
            source_file=os.path.join(package_dir, 'config', 'nav2_params.yaml'),
            root_key='',
            param_rewrites=odom_param_rewrites,
            convert_types=True,
        )

        gps_nav2_params = RewrittenYaml(
            source_file=os.path.join(package_dir, 'config', 'nav2_params.yaml'),
            root_key='',
            param_rewrites=common_param_rewrites,
            convert_types=True,
        )

        nav2_bringup_source = PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        )
        nav2_navigation_source = PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        )

        amcl_nav2_launch_arguments = [
            ('slam', 'False'),
            ('map', map_yaml),
            ('use_sim_time', use_sim_time),
            ('autostart', 'True'),
            ('use_composition', 'False'),
            ('use_respawn', 'True'),
            ('params_file', amcl_nav2_params),
        ]

        odom_nav2_launch_arguments = [
            ('use_sim_time', use_sim_time),
            ('autostart', 'True'),
            ('use_composition', 'False'),
            ('use_respawn', 'True'),
            ('params_file', odom_nav2_params),
        ]

        # AMCL localization uses the same front+rear scan coverage that was
        # used by Cartographer when building the map.
        amcl_scan_merger = Node(
            package='webots_ros2_tesla_slam',
            executable='dual_laser_scan_merger',
            name='dual_laser_scan_merger',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'front_scan_topic': '/scan_front',
                'rear_scan_topic': '/scan_rear',
                'output_topic': '/scan',
            }],
            condition=IfCondition(use_amcl),
        )

        # Odom localization deliberately keeps map and odom aligned. This mode
        # is useful when testing navigation behavior without global correction.
        def make_static_map_to_odom_node(condition):
            return Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom_static_tf',
                arguments=[
                    '--x', '0.0',
                    '--y', '0.0',
                    '--z', '0.0',
                    '--roll', '0.0',
                    '--pitch', '0.0',
                    '--yaw', '0.0',
                    '--frame-id', 'map',
                    '--child-frame-id', 'odom',
                ],
                condition=IfCondition(condition),
            )

        nav2_nodes_after_webots.append(amcl_scan_merger)
        nav2_nodes_after_webots.append(
            IncludeLaunchDescription(
                nav2_bringup_source,
                launch_arguments=amcl_nav2_launch_arguments,
                condition=IfCondition(use_amcl),
            )
        )
        nav2_nodes_without_webots.append(amcl_scan_merger)
        nav2_nodes_without_webots.append(
            IncludeLaunchDescription(
                nav2_bringup_source,
                launch_arguments=amcl_nav2_launch_arguments,
                condition=IfCondition(use_amcl),
            )
        )

        def make_odom_nodes():
            return [
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[odom_nav2_params],
                    condition=IfCondition(use_odom),
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': True,
                        'node_names': ['map_server'],
                    }],
                    condition=IfCondition(use_odom),
                ),
                make_static_map_to_odom_node(use_odom),
                IncludeLaunchDescription(
                    nav2_navigation_source,
                    launch_arguments=odom_nav2_launch_arguments,
                    condition=IfCondition(use_odom),
                ),
            ]

        nav2_nodes_after_webots.extend(make_odom_nodes())
        nav2_nodes_without_webots.extend(make_odom_nodes())

        gps_nav2_launch_arguments = [
            ('use_sim_time', use_sim_time),
            ('autostart', 'True'),
            ('use_composition', 'False'),
            ('use_respawn', 'True'),
            ('params_file', gps_nav2_params),
        ]
        gps_localization_params = os.path.join(
            package_dir, 'config', 'dual_ekf_navsat.yaml'
        )

        # GPS localization is assembled manually instead of using
        # bringup_launch.py so that robot_localization/navsat nodes own the
        # localization chain while Nav2 still runs the standard navigation set.
        def make_gps_nodes():
            return [
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[gps_nav2_params],
                    condition=IfCondition(gps_enabled),
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': True,
                        'node_names': ['map_server'],
                    }],
                    condition=IfCondition(gps_enabled),
                ),
                Node(
                    package='webots_ros2_tesla_slam',
                    executable='gps_navsat_relay',
                    name='gps_navsat_relay',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'input_topic': '/gps/fix',
                        'output_topic': '/gps/navsat',
                    }],
                    condition=IfCondition(gps_enabled),
                ),
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node_odom',
                    output='screen',
                    parameters=[
                        gps_localization_params,
                        {'use_sim_time': use_sim_time}
                    ],
                    remappings=[('odometry/filtered', 'odom')],
                    condition=IfCondition(gps_enabled),
                ),
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node_map',
                    output='screen',
                    parameters=[
                        gps_localization_params,
                        {
                            'use_sim_time': use_sim_time,
                            'publish_tf': True,
                        }
                    ],
                    remappings=[('odometry/filtered', 'odometry/global')],
                    condition=IfCondition(gps_enabled),
                ),
                Node(
                    package='robot_localization',
                    executable='navsat_transform_node',
                    name='navsat_transform',
                    output='screen',
                    parameters=[
                        gps_localization_params,
                        {'use_sim_time': use_sim_time}
                    ],
                    remappings=[
                        ('imu', '/imu'),
                        ('gps/fix', '/gps/navsat'),
                        ('odometry/filtered', '/odom'),
                        ('odometry/gps', '/odometry/gps'),
                    ],
                    condition=IfCondition(gps_enabled),
                ),
                IncludeLaunchDescription(
                    nav2_navigation_source,
                    launch_arguments=gps_nav2_launch_arguments,
                    condition=IfCondition(gps_enabled),
                ),
            ]

        gps_nodes_after_webots.extend(make_gps_nodes())
        gps_nodes_without_webots.extend(make_gps_nodes())

    cmd_vel_to_ackermann_after_webots = Node(
        package='webots_ros2_tesla_slam',
        executable='cmd_vel_to_ackermann',
        name='cmd_vel_to_ackermann',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wheelbase': 2.94,
            'invert_steering': True,
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
            'invert_steering': True,
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
            gps_nodes_after_webots +
            nav2_nodes_after_webots +
            [cmd_vel_to_ackermann_after_webots, rviz_after_webots]
        )
    )

    webots_group = GroupAction(
        actions=[
            SetEnvironmentVariable(
                name='WEBOTS_ROS2_TESLA_ODOM_TOPIC',
                value=driver_odom_topic,
            ),
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
            gps_nodes_without_webots +
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
        DeclareLaunchArgument(
            'localization',
            default_value='amcl',
            choices=['amcl', 'gps', 'odom'],
            description='Localization mode: amcl, gps or odom'
        ),
        webots_group,
        navigation_without_webots_group,
    ]

    return LaunchDescription(actions)
