from pathlib import Path
import unittest
import cv2
import numpy as np
from robomaster_example.map_processor import MapProcessor
from robomaster_example.path_planner import PathPlanner


class VisionPipelineTests(unittest.TestCase):
    def test_example_route_is_clear(self):
        image = cv2.imread(str(Path(__file__).resolve().parents[1] / 'examples/overhead.jpg'))
        grid, start, heading, goal, scale = MapProcessor().process_image(image)
        planner = PathPlanner()
        path = planner.a_star(grid, start, goal)
        self.assertIsNotNone(path)
        route = planner.simplify_path(grid, path)
        self.assertEqual(route[0], start)
        self.assertEqual(route[-1], goal)
        self.assertTrue(all(planner._is_line_free(grid, a, b) for a, b in zip(route, route[1:])))

    def test_corner_cutting_is_rejected(self):
        grid = np.array([[1, 0], [0, 1]])
        planner = PathPlanner()
        self.assertIsNone(planner.a_star(grid, (0, 0), (1, 1)))
        self.assertFalse(planner._is_line_free(grid, (0, 0), (1, 1)))
