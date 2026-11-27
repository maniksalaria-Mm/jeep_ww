"""Send a Nav2 goal from the terminal (navigation.launch.py must be running).

    ros2 run jeep_ww go_to summit        # top of the mountain
    ros2 run jeep_ww go_to turing        # campus world: turing, babbage, picasso, square_one, ...
    ros2 run jeep_ww go_to bend2         # a named place (see PLACES)
    ros2 run jeep_ww go_to 63 30 90      # x y yaw(deg) in the map frame
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

# x, y, yaw (deg) -- all facing the uphill direction of travel
PLACES = {
    'ramp1': (35.0, 0.0, 0.0),
    'bend1': (63.0, 4.0, 90.0),
    'ramp2': (63.0, 30.0, 90.0),
    'bend2': (55.0, 56.0, 180.0),
    'ramp3': (30.0, 56.0, 180.0),
    'bend3': (7.0, 60.0, 90.0),
    'ramp4': (7.0, 85.0, 90.0),
    'summit': (7.0, 116.0, 90.0),
    # campus world (chitkara_campus.sdf) - poses on the road next to each block
    'turing': (120.0, 0.0, 0.0),
    'babbage': (130.0, 52.0, 90.0),
    'fleming': (130.0, 84.0, 90.0),
    'bloom': (112.0, 100.0, 180.0),
    'corbusier': (78.0, 100.0, 180.0),
    'picasso': (36.0, 100.0, 180.0),
    'galileo': (10.0, 60.0, -90.0),
    'square_one': (38.0, -100.0, 0.0),
    'edison': (82.0, -100.0, 0.0),
    'newton': (118.0, -100.0, 0.0),
    'tesla': (130.0, -86.0, 90.0),
    'parking': (130.0, -46.0, 90.0),
    'main_gate': (25.0, 0.0, 180.0),
}


def main():
    args = sys.argv[1:]
    args = [a for a in args if not a.startswith('--ros-args')]
    if len(args) == 1 and args[0] in PLACES:
        x, y, yaw = PLACES[args[0]]
    elif len(args) == 3:
        x, y, yaw = (float(a) for a in args)
    else:
        print(__doc__)
        print('places:', ', '.join(PLACES))
        return

    rclpy.init()
    node = rclpy.create_node('go_to')
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    print('waiting for Nav2 ...')
    if not client.wait_for_server(timeout_sec=60.0):
        print('Nav2 is not running: start "ros2 launch jeep_ww navigation.launch.py" first')
        return

    goal = NavigateToPose.Goal()
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(math.radians(yaw) / 2)
    pose.pose.orientation.w = math.cos(math.radians(yaw) / 2)
    goal.pose = pose

    state = {'last': -10.0}

    def feedback(msg):
        fb = msg.feedback
        t = fb.navigation_time.sec
        if t - state['last'] >= 10:
            state['last'] = t
            p = fb.current_pose.pose.position
            print(f'  {t:4d}s  at ({p.x:5.1f}, {p.y:5.1f})  {fb.distance_remaining:5.1f} m to go')

    print(f'goal: x={x} y={y} yaw={yaw} deg')
    send = client.send_goal_async(goal, feedback_callback=feedback)
    rclpy.spin_until_future_complete(node, send)
    handle = send.result()
    if not handle.accepted:
        print('goal rejected: is Nav2 fully started ("Managed nodes are active")?')
        return
    result = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, result)
    except KeyboardInterrupt:
        handle.cancel_goal_async()
        print('cancelled')
        return
    status = result.result().status
    print('arrived' if status == 4 else f'failed (status {status}, error {result.result().result.error_code})')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
