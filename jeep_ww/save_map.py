"""Save the SLAM map (mapping.launch.py must be running).

    ros2 run jeep_ww save_map                 # -> ~/bot_ws/src/jeep_ww/maps/my_map.yaml/.pgm
    ros2 run jeep_ww save_map ~/maps/hill     # any path, without extension
"""
import os
import subprocess
import sys


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--ros-args')]
    default = os.path.expanduser('~/bot_ws/src/jeep_ww/maps/my_map')
    path = os.path.expanduser(args[0]) if args else default
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', path,
           '--ros-args', '-p', 'use_sim_time:=true', '-p', 'save_map_timeout:=20.0']
    code = subprocess.call(cmd)
    if code == 0:
        print(f'\nmap saved: {path}.yaml')
        print(f'navigate with it:  ros2 launch jeep_ww navigation.launch.py map:={path}.yaml')
        print('(rebuild with colcon if you saved it inside the package and want it installed)')
    sys.exit(code)


if __name__ == '__main__':
    main()
