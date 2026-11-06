"""Camera-feedback waypoint control for the fixed overhead-camera scenes."""
import math
import time
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from .map_processor import MapProcessor
from .path_planner import PathPlanner


def tracking_command(position, heading, target, scale, max_speed=0.35):
    """Return forward speed, turn rate and distance in pixels."""
    dx, dy = target[0] - position[0], position[1] - target[1]
    distance = math.hypot(dx, dy)
    error = math.atan2(math.sin(math.atan2(dy, dx) - heading),
                       math.cos(math.atan2(dy, dx) - heading))
    angular = max(-1.0, min(1.0, 2.5 * error))
    linear = (min(max_speed, max(0.06, 1.5 * distance * scale)) * math.cos(error) ** 2
              if abs(error) < 0.35 else 0.0)
    return linear, angular, distance


class RobomasterControllerNode(Node):
    def __init__(self):
        super().__init__('robomaster_controller_node')
        for key, value in [('update_mode', False), ('margin', 6),
                           ('camera_topic', '/rm1/camera/image_color'),
                           ('update_interval', 3.0)]:
            self.declare_parameter(key, value)
        self.bridge = CvBridge()
        self.processor = MapProcessor(self.get_parameter('margin').value)
        self.planner = PathPlanner()
        self.path = []
        self.index = 0
        self.last_frame = -math.inf
        self.last_map = -math.inf
        self.last_plan = -math.inf
        self.completed = False
        self.pose = None
        self.grid = None
        self.output = Path('outputs/navigation')
        self.output.mkdir(parents=True, exist_ok=True)
        self.plan_count = 0
        self.last_failure_image = -math.inf
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.camera_subscriber = self.create_subscription(
            Image, self.get_parameter('camera_topic').value, self.camera_callback, 1)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Waiting for overhead images; using camera-feedback control')

    def stop_robot(self):
        self.cmd_vel_publisher.publish(Twist())

    def camera_callback(self, msg):
        now = time.monotonic()
        if now - self.last_frame < 0.1:
            return
        image = None
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            position, heading = self.processor._find_robot(image)
            self.pose = position, heading
            self.last_frame = now
            # Refresh obstacle geometry at 2 Hz. Pose feedback continues at 10 Hz.
            if now - self.last_map >= 0.5:
                grid, _, _, goal, _ = self.processor.process_image(image)
                self.grid, self.goal, self.image = grid, goal, image
                self.last_map = now
        except (ValueError, cv2.error) as exc:
            self.stop_robot()
            if image is not None and now - self.last_failure_image > 2.0:
                cv2.imwrite(str(self.output / 'localization_failure.jpg'), image)
                self.last_failure_image = now
            self.pose = None
            self.path = []
            self.get_logger().error(f'Localization/map unavailable; stopped: {exc}', throttle_duration_sec=2.0)

    def plan_path(self):
        self.stop_robot()
        self.path = []
        start = self.pose[0]
        route = self.planner.a_star(self.grid, start, self.goal)
        self.last_plan = time.monotonic()
        if route is None:
            self.get_logger().warning('No path with footprint clearance; staying stopped')
            return
        self.path = self.planner.simplify_path(self.grid, route)
        self.planned_goal = self.goal
        self.index = 1
        self.plan_count += 1
        overlay = self.processor.visualize(self.image, self.grid, start, self.goal, self.path)
        cv2.imwrite(str(self.output / f'path_{self.plan_count:03d}.jpg'), overlay)
        self.get_logger().info(f'Planned {len(self.path)} waypoints from measured position {start}')

    def control_loop(self):
        now = time.monotonic()
        if self.completed or self.pose is None or self.grid is None or now - self.last_frame > 0.5:
            self.stop_robot()
            return
        periodic = self.get_parameter('update_mode').value and (
            now - self.last_plan >= self.get_parameter('update_interval').value)
        if periodic and self.path:
            # Map updates need not interrupt a still-valid route.
            remaining = [self.pose[0]] + self.path[self.index:]
            changed = math.dist(self.goal, self.planned_goal) > 3
            blocked = any(not self.planner._is_line_free(self.grid, a, b)
                          for a, b in zip(remaining, remaining[1:]))
            self.last_plan = now
            if changed or blocked:
                self.plan_path()
                return
        if not self.path and now - self.last_plan >= 1.0:
            self.plan_path()
            return
        if not self.path:
            self.stop_robot()
            return
        position, heading = self.pose
        while self.index < len(self.path) and math.dist(position, self.path[self.index]) <= 3:
            self.index += 1
        if self.index >= len(self.path):
            self.completed = True
            self.stop_robot()
            self.get_logger().info('Goal reached using measured camera position')
            return
        # Skip tiny intermediate corner waypoints only when the direct line
        # from the measured position clears the inflated obstacle map.
        for candidate in range(len(self.path) - 1, self.index, -1):
            if self.planner._is_line_free(self.grid, position, self.path[candidate]):
                self.index = candidate
                break
        target = self.path[self.index]
        if not self.planner._is_line_free(self.grid, position, target):
            self.stop_robot()
            self.path = []
            return
        linear, angular, _ = tracking_command(position, heading, target, self.processor.scale[0])
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(linear), float(angular)
        self.cmd_vel_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RobomasterControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
