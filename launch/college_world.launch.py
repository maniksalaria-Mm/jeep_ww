"""Open the college floor world (Mechatronics department, floor 2) in Gazebo and its map in RViz.

ros2 launch jeep_ww college_world.launch.py               # Gazebo world + map in RViz
ros2 launch jeep_ww college_world.launch.py rviz:=false
ros2 launch jeep_ww college_world.launch.py gazebo:=false  # map only

Spawn your own small robot at (0, 0) facing +x; the map is in the same frame as the world.
(The jeep_ww SUV is too big for indoor corridors.)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('jeep_ww')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world = os.path.join(pkg, 'worlds', 'college_corridor.sdf')
    map_yaml = os.path.join(pkg, 'maps', 'college_corridor.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('gazebo', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'-r -v 2 {world}', 'on_exit_shutdown': 'true'}.items(),
            condition=IfCondition(LaunchConfiguration('gazebo')),
        ),

        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            parameters=[{'yaml_filename': map_yaml}], output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
            parameters=[{'autostart': True, 'node_names': ['map_server']}], output='screen',
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz2', executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'config', 'college_map.rviz')], output='screen',
        ),
    ])
