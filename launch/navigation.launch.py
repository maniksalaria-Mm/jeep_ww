"""Autonomous navigation with Nav2.

ros2 launch jeep_ww navigation.launch.py                    # ready-made map, click goals in RViz
ros2 launch jeep_ww navigation.launch.py goal:=summit       # also drives to the summit by itself
ros2 launch jeep_ww navigation.launch.py map:=$HOME/bot_ws/src/jeep_ww/maps/my_map.yaml
ros2 launch jeep_ww navigation.launch.py slam:=True         # map while navigating (goals must be
                                                            # inside the area already mapped)
Send goals from a terminal:  ros2 run jeep_ww go_to summit
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationNotEquals, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('jeep_ww')
    nav2 = get_package_share_directory('nav2_bringup')
    slam_toolbox = get_package_share_directory('slam_toolbox')

    slam = LaunchConfiguration('slam')
    # point bt_navigator at the car behaviour tree shipped in this package
    params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites={'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml':
                        os.path.join(pkg, 'config', 'navigate_car.xml')},
        convert_types=True)

    nav_with_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'slam': 'False',
            'use_localization': 'True',
            'params_file': params,
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
        condition=UnlessCondition(slam),
    )

    nav_with_slam = GroupAction(condition=IfCondition(slam), actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': LaunchConfiguration('slam_params'),
                'use_sim_time': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'params_file': params,
                'use_sim_time': 'true',
                'autostart': 'true',
                'use_composition': 'False',
            }.items(),
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='mountain_road.sdf'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('slam_params', default_value=os.path.join(pkg, 'config', 'slam_params.yaml')),
        DeclareLaunchArgument('slam', default_value='False',
                              description='True = build the map while navigating'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'mountain_road.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(pkg, 'config', 'navigation.rviz')),
        DeclareLaunchArgument('goal', default_value='none',
                              description='place to drive to automatically, e.g. summit'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'), 'rviz': 'false',
                              'headless': LaunchConfiguration('headless')}.items(),
        ),

        TimerAction(period=8.0, actions=[nav_with_map, nav_with_slam]),

        # optional automatic goal, sent once Nav2 has had time to start
        TimerAction(period=40.0, actions=[
            Node(
                condition=LaunchConfigurationNotEquals('goal', 'none'),
                package='jeep_ww',
                executable='go_to',
                arguments=[LaunchConfiguration('goal')],
                output='screen',
            ),
        ]),

        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz2',
            executable='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
