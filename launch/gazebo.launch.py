"""Gazebo (Harmonic) + jeep_ww SUV.

ros2 launch jeep_ww gazebo.launch.py                       # mountain road (default)
ros2 launch jeep_ww gazebo.launch.py world:=empty.world    # any file in jeep_ww/worlds
ros2 launch jeep_ww gazebo.launch.py rviz:=true
ros2 launch jeep_ww gazebo.launch.py headless:=true      # no Gazebo window
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('jeep_ww')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('rviz')
    x, y, z, yaw = (LaunchConfiguration(n) for n in ('x', 'y', 'z', 'yaw'))

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg, 'urdf', 'jeep_ww.urdf.xacro')]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='mountain_road.sdf',
                              description='World file name inside jeep_ww/worlds'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = no Gazebo window'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.1'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        # lets the world find the mountain meshes as model://jeep_ww/meshes/...
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': ['-r -v 3 ', PathJoinSubstitution([pkg, 'worlds', world])],
                'on_exit_shutdown': 'true',
            }.items(),
            condition=UnlessCondition(LaunchConfiguration('headless')),
        ),
        # headless:=true -> physics + sensors only, no Gazebo window (faster on WSL)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': ['-r -s --headless-rendering -v 3 ', PathJoinSubstitution([pkg, 'worlds', world])],
                'on_exit_shutdown': 'true',
            }.items(),
            condition=IfCondition(LaunchConfiguration('headless')),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen',
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'jeep_ww',
                       '-x', x, '-y', y, '-z', z, '-Y', yaw],
            output='screen',
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': os.path.join(pkg, 'config', 'gz_bridge.yaml'),
                         'use_sim_time': True}],
            output='screen',
        ),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'config', 'mapping.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
