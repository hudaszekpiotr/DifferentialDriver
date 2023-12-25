# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node, SetParameter, LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('startup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    slam_mode = LaunchConfiguration('slam_mode')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['map_server',
                       'amcl',
                       'controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower']
    lifecycle_nodes_slam = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower']


    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    urdf_pkg = FindPackageShare(package='robot_urdf').find('robot_urdf')
    model_path = os.path.join(urdf_pkg, 'urdf_model/robot.urdf')

    nav2_config = os.path.join(
        get_package_share_directory('startup'),
        'config',
        'nav2_params.yaml'
    )

    imu_filter_madgwick_config = os.path.join(
        get_package_share_directory('startup'),
        'config',
        'imu.yaml'
    )

    slam_config = os.path.join(
        get_package_share_directory('startup'),
        'config',
        'slam.yaml'
    )

    ekf_config = os.path.join(
        get_package_share_directory('startup'),
        'config',
        'ekf.yaml'
    )

    twist_mux_config = os.path.join(
        get_package_share_directory('startup'),
        'config',
        'twist_mux.yaml'
    )

    joy_config = os.path.join(
        get_package_share_directory('startup'),
        'config',
        'joy.yaml'
    )

    twist_joy_config = os.path.join(
        get_package_share_directory('startup'),
        'config',
        'twist_joy.yaml'
    )

    camera_config = os.path.join(
        get_package_share_directory('startup'),
        'config',
        'camera.yaml'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_slam_mode_cmd = DeclareLaunchArgument(
        'slam_mode', default_value='False',
        description='Use SLAM')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    load_composable_nodes  = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[nav2_config,
                            {'use_sim_time': use_sim_time}],
                condition=IfCondition(PythonExpression(['not ', slam_mode])),
                remappings = [('odom', 'odometry/filtered')],
            ),
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[nav2_config,
                            {'use_sim_time': use_sim_time}],
                condition=IfCondition(PythonExpression(['not ', slam_mode])),
                remappings = [('odom', 'odometry/filtered')],
            ),
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[nav2_config],
                remappings= [('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[nav2_config]),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[nav2_config]),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[nav2_config]),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[nav2_config]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'autostart': autostart},
                            {'node_names': lifecycle_nodes_slam}],
                condition=IfCondition(slam_mode)
                ),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'autostart': autostart},
                            {'node_names': lifecycle_nodes}],
                condition=IfCondition(PythonExpression(['not ', slam_mode]))
                ),
            ])
        ]
    )

    ld = LaunchDescription()

    lidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD19'},
            {'topic_name': 'scan'},
            {'frame_id': 'base_laser'},
            {'port_name': '/dev/ttyUSB0'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0}
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
        'robot_description': Command(['xacro ', model_path])}]
    )
    imu_node_cpp = Node(
        package='imu_cpp',
        executable='main',
        name='imu_node'
    )
    motors_node_cpp = Node(
        package='motors_driver_cpp',
        executable='main',
        name='motors_driver_node'
    )
    odometry_node_cpp = Node(
        package='odometry_cpp',
        executable='main',
        name='odometry_node'
    )
    twist_to_wheels_speed_node_cpp = Node(
        package='twist_to_wheels_speed_cpp',
        executable='main',
        name='twist_to_wheels_speed_node'
    )
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[imu_filter_madgwick_config],
        remappings=[('/tf', 'unused/madgwick_tf')]  # workaround as publish_tf param is ignored
    )
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        #  arguments=['--ros-args', '--log-level', 'debug'],
    )
    slam_node = Node(
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(slam_mode),
    )
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings={('/cmd_vel_out', 'cmd_vel')},
        parameters=[twist_mux_config]
    )
    joy_node = Node(
        package='joy',
        name='joy_node',
        executable='joy_node',
        parameters=[joy_config],
    )
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        name='TeleopTwistJoy',
        executable='teleop_node',
        remappings=[('cmd_vel', 'cmd_vel_joy')],
        parameters=[twist_joy_config],
    )
    screen_node = Node(
        package='screen',
        name='screen_node',
        executable='main'
    )
    camera_node = Node(
        package='v4l2_camera',
        name='v4l2_camera_node',
        executable='v4l2_camera_node',
        parameters=[camera_config],
    )
    cliff_sensor_node_cpp = Node(
        package='cliff_sensor_cpp',
        name='cliff_sensor_node',
        executable='main'
    )
    ultrasonic_sensor_node_cpp = Node(
        package='ultrasonic_sensor_cpp',
        name='ultrasonic_sensor_node',
        executable='main'
    )

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)


    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_slam_mode_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    ld.add_action(imu_node_cpp)
    ld.add_action(lidar_node)
    ld.add_action(motors_node_cpp)
    ld.add_action(odometry_node_cpp)
    ld.add_action(twist_to_wheels_speed_node_cpp)
    ld.add_action(imu_filter_node)
    ld.add_action(ekf_node)
    ld.add_action(slam_node)
    ld.add_action(twist_mux_node)
    ld.add_action(joy_node)
    ld.add_action(teleop_twist_joy_node)
    ld.add_action(robot_state_publisher_node)
    #ld.add_action(screen_node)
    #ld.add_action(camera_node)
    #ld.add_action(cliff_sensor_node_cpp)
    #ld.add_action(ultrasonic_sensor_node_cpp)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_composable_nodes )

    return ld

