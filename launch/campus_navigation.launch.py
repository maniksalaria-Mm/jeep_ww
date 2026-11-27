"""Autonomous navigation (Nav2) for the SUV in the university-campus world.

ros2 launch jeep_ww campus_navigation.launch.py                 # click "2D Goal Pose" in RViz
ros2 launch jeep_ww campus_navigation.launch.py goal:=turing    # also drives to the Turing Block
ros2 launch jeep_ww campus_navigation.launch.py traffic:=false  # no moving people and cars
ros2 launch jeep_ww campus_navigation.launch.py map:=$HOME/bot_ws/src/jeep_ww/maps/my_campus_map.yaml

More goals (2nd terminal):  ros2 run jeep_ww go_to picasso
Places: turing, babbage, fleming, bloom, corbusier, picasso, galileo, square_one, edison,
        newton, tesla, parking, main_gate

Localisation: slam_toolbox (it tracks the SUV far better than AMCL on the open campus).
Planning: both costmaps use the roads-only map on /nav_map, so the SUV knows the whole road
network from the start and stays on the roads.
localization:=amcl uses AMCL on maps/chitkara_campus.yaml instead.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('jeep_ww')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'chitkara_campus.yaml')),
        DeclareLaunchArgument('roads_map', default_value=os.path.join(pkg, 'maps', 'chitkara_campus_roads.yaml')),
        DeclareLaunchArgument('goal', default_value='none'),
        DeclareLaunchArgument('localization', default_value='slam', description='slam or amcl'),
        DeclareLaunchArgument('traffic', default_value='true',
                              description='true = moving people and cars, false = empty campus'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # roads-only map for the costmaps, published on /nav_map
        Node(package='nav2_map_server', executable='map_server', name='nav_map_server',
             parameters=[{'yaml_filename': LaunchConfiguration('roads_map'), 'topic_name': 'nav_map',
                          'frame_id': 'map', 'use_sim_time': True}],
             output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_nav_map',
             parameters=[{'autostart': True, 'node_names': ['nav_map_server'], 'use_sim_time': True}],
             output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'world': PythonExpression(["'chitkara_campus_traffic.sdf' if '", LaunchConfiguration('traffic'), "' == 'true' else 'chitkara_campus.sdf'"]),
                'map': LaunchConfiguration('map'),
                'params_file': os.path.join(pkg, 'config', 'nav2_params_campus.yaml'),
                'goal': LaunchConfiguration('goal'),
                'slam': PythonExpression(["'True' if '", LaunchConfiguration('localization'), "' == 'slam' else 'False'"]),
                'slam_params': os.path.join(pkg, 'config', 'slam_params_campus.yaml'),
                'headless': LaunchConfiguration('headless'),
                'rviz': LaunchConfiguration('rviz'),
                'rviz_config': os.path.join(pkg, 'config', 'campus_navigation.rviz'),
            }.items(),
        ),
    ])
