"""Drive the jeep_ww SUV along the mountain road by itself (pure pursuit on /odom).

Use it with mapping.launch.py to build the map automatically:
    ros2 run jeep_ww auto_drive                       # start -> summit
    ros2 run jeep_ww auto_drive --ros-args -p route:=down      # summit -> start
    ros2 run jeep_ww auto_drive --ros-args -p route:=up_down   # up and back down
    ros2 run jeep_ww auto_drive --ros-args -p speed:=3.0
    ros2 run jeep_ww auto_drive --ros-args -p route:=campus    # tour of the campus world
    ros2 run jeep_ww auto_drive --ros-args -p route:=campus -p stop_for_obstacles:=true
        (stops and waits when the front bumper lidar sees a person or car in its path;
         keep it off on the mountain, where the low lidar sees the ramps ahead)

The route is the road centre line of worlds/mountain_road.sdf (the SUV spawns at 0,0
facing +x, so map = odom = world). The SUV is steered from the map -> base_footprint
transform (SLAM or AMCL corrected) when it exists, otherwise from /odom.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

UP = [(0, 0), (50, 0), (56, 0.8), (60.5, 3.2), (63, 7.5), (63, 48), (62.2, 52.5), (59, 55.2),
      (54, 56), (16, 56), (11.5, 56.8), (8.3, 59.5), (7, 64), (7, 116)]


def _arc(cx, cy, r, a0, a1, step_deg=10):
    n = max(2, int(abs(a1 - a0) / step_deg))
    return [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
             cy + r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]


# Turn around on the summit plateau (anticlockwise loop, then a right-hand bend into the road)
SUMMIT_LOOP = [(7, 116)] + _arc(0.5, 119, 7.0, 0, 270) + _arc(0.5, 105.5, 6.5, 90, 0)
DOWN = SUMMIT_LOOP + [(7, 100)] + UP[::-1][1:-1] + [(5, 0)]


# Campus world (chitkara_campus.sdf): boulevard to the Turing Block, round the whole road loop
# (north on the back road, west on the north road, south on the front road, east on the south
# road), then back down the boulevard towards the gate. Corners are cut inside the turning circles.
CAMPUS = [(0, 0), (120, 0), (130, 9), (130, 91), (121, 100), (19, 100), (10, 91), (10, -91),
          (19, -100), (121, -100), (130, -91), (130, -9), (121, 0), (30, 0)]


def densify(points, step=0.5):
    out = []
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
        out += [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n)]
    out.append(points[-1])
    return out


class AutoDrive(Node):

    def __init__(self):
        super().__init__('auto_drive')
        route = self.declare_parameter('route', 'up').value
        self.speed = float(self.declare_parameter('speed', 2.0, ParameterDescriptor(dynamic_typing=True)).value)
        self.lookahead = float(self.declare_parameter('lookahead', 5.0).value)
        routes = {'up': UP, 'down': DOWN, 'up_down': UP + DOWN[1:], 'campus': CAMPUS}
        if route not in routes:
            raise ValueError(f'route must be one of {list(routes)}')
        self.path = densify(routes[route])
        self.pose = None
        self.idx = None
        self.done = False
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.stop_for_obstacles = bool(self.declare_parameter('stop_for_obstacles', False).value)
        self.blocked = False
        self.heading_to_target = 0.0    # bearing of the pure-pursuit target, vehicle frame
        if self.stop_for_obstacles:
            self.create_subscription(LaserScan, 'front_scan', self.on_scan, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.source = None
        self.timer = self.create_timer(0.05, self.step)
        self.last_log = 0.0
        self.get_logger().info(f'route "{route}", {len(self.path)} points, {self.speed} m/s. Waiting for /odom...')

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def on_scan(self, msg):
        """Blocked if any front-lidar hit lies in the lane ahead, measured along the direction the
        SUV is steering towards (so a hedge beside the road does not block a turn)."""
        reach = 3.0 + 1.5 * self.speed
        blocked = False
        ca, sa = math.cos(self.heading_to_target), math.sin(self.heading_to_target)
        a = msg.angle_min
        for r in msg.ranges:
            if 1.0 < r < reach:                       # < 1 m: the SUV's own bumper
                px, py = r * math.cos(a), r * math.sin(a)
                x, y = px * ca + py * sa, -px * sa + py * ca
                if px > 0.2 and x > 0.0 and abs(y) < 1.4:
                    blocked = True
                    break
            a += msg.angle_increment
        if blocked and not self.blocked:
            self.get_logger().info('obstacle ahead - waiting')
        elif self.blocked and not blocked:
            self.get_logger().info('path clear - driving on')
        self.blocked = blocked

    def current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time(), Duration(seconds=0.05))
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            src, pose = 'map', (t.transform.translation.x, t.transform.translation.y, yaw)
        except TransformException:
            src, pose = 'odom', self.pose
        if src != self.source and pose is not None:
            self.source = src
            self.get_logger().info(f'steering from the {src} frame')
        return pose

    def step(self):
        if self.done:
            return
        pose = self.current_pose()
        if pose is None:
            return
        x, y, yaw = pose
        if self.idx is None:
            # start from the route point nearest to where the SUV is now (first 40 m only)
            near = range(min(len(self.path), 80))
            self.idx = min(near, key=lambda i: math.hypot(self.path[i][0] - x, self.path[i][1] - y))
            self.get_logger().info(f'starting at ({x:.1f}, {y:.1f})')
        while self.idx < len(self.path) - 1 and \
                math.hypot(self.path[self.idx][0] - x, self.path[self.idx][1] - y) < self.lookahead:
            self.idx += 1
        gx, gy = self.path[self.idx]
        last = self.idx == len(self.path) - 1
        dist_end = math.hypot(gx - x, gy - y)
        if last and dist_end < 1.5:
            self.pub.publish(Twist())
            self.get_logger().info(f'arrived at ({x:.1f}, {y:.1f})')
            self.done = True
            return
        alpha = math.atan2(gy - y, gx - x) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        self.heading_to_target = max(-0.8, min(0.8, alpha))
        if self.blocked:
            self.pub.publish(Twist())
            return
        v = self.speed if not last else max(0.5, min(self.speed, dist_end / 3.0))
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = v * 2.0 * math.sin(alpha) / self.lookahead
        self.pub.publish(cmd)
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_log > 10.0:
            self.last_log = now
            pct = 100.0 * self.idx / (len(self.path) - 1)
            self.get_logger().info(f'at ({x:.1f}, {y:.1f})  {pct:.0f}% of the route')


def main():
    rclpy.init()
    node = AutoDrive()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
