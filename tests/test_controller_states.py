import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
from robomaster_example.path_planner import PathPlanner
try:
    from robomaster_example.robomaster_controller_node import RobomasterControllerNode, tracking_command
except ModuleNotFoundError as exc:
    if exc.name in {'rclpy', 'cv_bridge', 'geometry_msgs', 'sensor_msgs'}:
        raise unittest.SkipTest('Controller tests require ROS 2 and cv_bridge') from exc
    raise


class NavigationTests(unittest.TestCase):
    def test_no_diagonal_corner_cutting(self):
        grid = np.array([[1, 0], [0, 1]])
        planner = PathPlanner()
        self.assertIsNone(planner.a_star(grid, (0, 0), (1, 1)))
        self.assertFalse(planner._is_line_free(grid, (0, 0), (1, 1)))

    def test_shortcuts_preserve_clearance(self):
        grid = np.ones((60, 60), dtype=np.uint8)
        grid[10:50, 25:35] = 0
        planner = PathPlanner()
        path = planner.a_star(grid, (5, 30), (55, 30))
        simplified = planner.simplify_path(grid, path, max_points=2)
        self.assertGreater(len(simplified), 2)
        self.assertTrue(all(planner._is_line_free(grid, a, b) for a, b in zip(simplified, simplified[1:])))

    def test_turn_before_driving(self):
        speed, turn, _ = tracking_command((0, 0), math.pi / 2, (100, 0), 0.015)
        self.assertEqual(speed, 0)
        self.assertLess(turn, 0)

    def test_slow_down_near_waypoint(self):
        near, _, _ = tracking_command((0, 0), 0, (4, 0), 0.015)
        far, _, _ = tracking_command((0, 0), 0, (100, 0), 0.015)
        self.assertLess(near, far)
        self.assertLessEqual(far, 0.35)

    def test_lost_pose_stops(self):
        node = SimpleNamespace(completed=False, pose=None, stop_robot=Mock())
        RobomasterControllerNode.control_loop(node)
        node.stop_robot.assert_called_once()

    def test_stale_frame_stops(self):
        node = SimpleNamespace(completed=False, pose=((0, 0), 0), grid=np.ones((2, 2)), last_frame=-math.inf, stop_robot=Mock())
        RobomasterControllerNode.control_loop(node)
        node.stop_robot.assert_called_once()

    def test_failed_plan_keeps_path_empty(self):
        node = SimpleNamespace(stop_robot=Mock(), path=[(0, 0)], pose=((0, 0), 0),
                               grid=np.zeros((3, 3)), goal=(2, 2), planner=PathPlanner(),
                               get_logger=Mock(return_value=Mock()))
        RobomasterControllerNode.plan_path(node)
        self.assertEqual(node.path, [])
        node.stop_robot.assert_called_once()

    def test_goal_requires_measured_arrival(self):
        import time
        node = SimpleNamespace(completed=False, pose=((10, 10), 0), grid=np.ones((20, 20)),
                               last_frame=time.monotonic(), last_plan=time.monotonic(),
                               get_parameter=Mock(return_value=SimpleNamespace(value=False)),
                               path=[(0, 0), (10, 10)], index=1, stop_robot=Mock(),
                               get_logger=Mock(return_value=Mock()))
        RobomasterControllerNode.control_loop(node)
        self.assertTrue(node.completed)
        node.stop_robot.assert_called_once()
