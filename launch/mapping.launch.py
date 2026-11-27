"""Build a map of the mountain road with slam_toolbox.

ros2 launch jeep_ww mapping.launch.py                   # drive yourself (teleop)
ros2 launch jeep_ww mapping.launch.py auto:=true        # SUV drives up and back down by itself
ros2 launch jeep_ww mapping.launch.py auto:=true route:=up

Drive yourself:  ros2 run teleop_twist_keyboard teleop_twist_keyboard
Save the map:    ros2 run jeep_ww save_map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('jeep_ww')
    slam_toolbox = get_package_share_directory('slam_toolbox')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='mountain_road.sdf'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('auto', default_value='false',
                              description='true = drive the route automatically'),
        DeclareLaunchArgument('route', default_value='up_down', description='up, down, up_down or campus'),
        DeclareLaunchArgument('speed', default_value='2.0', description='auto-drive speed (m/s)'),
        DeclareLaunchArgument('stop_for_obstacles', default_value='false',
                              description='auto-drive waits for people / cars seen by the front lidar'),
        DeclareLaunchArgument('slam_params', default_value=os.path.join(pkg, 'config', 'slam_params.yaml')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'), 'rviz': 'false',
                              'headless': LaunchConfiguration('headless')}.items(),
        ),

        # give Gazebo a few seconds to load the world and spawn the SUV
        TimerAction(period=6.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')),
                launch_arguments={
                    'slam_params_file': LaunchConfiguration('slam_params'),
                    'use_sim_time': 'true',
                }.items(),
            ),
        ]),

        # automatic driver starts once SLAM is publishing the map frame
        TimerAction(period=15.0, actions=[
            Node(
                condition=IfCondition(LaunchConfiguration('auto')),
                package='jeep_ww',
                executable='auto_drive',
                parameters=[{'route': LaunchConfiguration('route'), 'speed': LaunchConfiguration('speed'),
                             'stop_for_obstacles': LaunchConfiguration('stop_for_obstacles'),
                             'use_sim_time': True}],
                output='screen',
            ),
        ]),

        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'config', 'mapping.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
