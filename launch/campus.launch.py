"""SUV in the university-campus world, with SLAM mapping and RViz.

ros2 launch jeep_ww campus.launch.py                 # drive yourself with teleop, map builds as you go
ros2 launch jeep_ww campus.launch.py auto:=true      # SUV tours the whole campus by itself
ros2 launch jeep_ww campus.launch.py auto:=true speed:=4.0
ros2 launch jeep_ww campus.launch.py headless:=true  # no Gazebo window (RViz still opens)
ros2 launch jeep_ww campus.launch.py traffic:=false  # campus without moving people and cars

With traffic (default) people walk and cross the roads and cars and a bus drive round; the tour
stops and waits when the front bumper lidar sees someone in its path.

Drive yourself:  ros2 run teleop_twist_keyboard teleop_twist_keyboard
Save the map:    ros2 run jeep_ww save_map ~/bot_ws/src/jeep_ww/maps/my_campus_map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg = get_package_share_directory('jeep_ww')
    return LaunchDescription([
        DeclareLaunchArgument('auto', default_value='false', description='true = tour the campus by itself'),
        DeclareLaunchArgument('speed', default_value='3.0', description='tour speed (m/s)'),
        DeclareLaunchArgument('traffic', default_value='true',
                              description='true = moving people and cars, false = empty campus'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'mapping.launch.py')),
            launch_arguments={
                'world': PythonExpression(["'chitkara_campus_traffic.sdf' if '", LaunchConfiguration('traffic'), "' == 'true' else 'chitkara_campus.sdf'"]),
                'slam_params': os.path.join(pkg, 'config', 'slam_params_campus.yaml'),
                'auto': LaunchConfiguration('auto'),
                'route': 'campus',
                'speed': LaunchConfiguration('speed'),
                'stop_for_obstacles': 'true',
                'headless': LaunchConfiguration('headless'),
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
    ])
