"""Look at the SUV model in RViz (no simulation).

ros2 launch jeep_ww display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('jeep_ww')
    gui = LaunchConfiguration('gui')
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg, 'urdf', 'jeep_ww.urdf.xacro')]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Show sliders to turn the wheels / steering'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(condition=UnlessCondition(gui), package='joint_state_publisher',
             executable='joint_state_publisher'),
        Node(condition=IfCondition(gui), package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg, 'config', 'display.rviz')], output='screen'),
    ])
