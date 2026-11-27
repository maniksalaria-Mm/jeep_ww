"""Stop every leftover Gazebo / Nav2 / SLAM / RViz process from earlier runs.

    ros2 run jeep_ww stop_sim

Run it when a launch fails with "Goal was rejected", "jump back in time" or a
second /clock publisher: those come from an old run that is still alive.
"""
import subprocess
import time

PATTERNS = [
    'ros2 launch', 'gz sim', 'gz-sim', 'ruby', 'parameter_bridge', 'robot_state_publisher',
    'rviz2', 'slam_toolbox', 'controller_server', 'planner_server', 'smoother_server',
    'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother',
    'collision_monitor', 'docking_server', 'route_server', 'lifecycle_manager', 'map_server',
    'amcl', 'component_container', 'joint_state_publisher', 'jeep_ww/auto_drive',
    'ros_gz_sim/create', 'teleop_twist_keyboard',
]


def main():
    for sig in ('-INT', '-KILL'):
        for p in PATTERNS:
            subprocess.call(['pkill', sig, '-f', p], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2.0)
    subprocess.call(['ros2', 'daemon', 'stop'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print('all simulation processes stopped')


if __name__ == '__main__':
    main()
