import math
import unittest
import cv2
import numpy as np
from robomaster_example.map_processor import MapProcessor


class MarkerDetectionTests(unittest.TestCase):
    def scene(self):
        image = np.full((160, 160, 3), 255, dtype=np.uint8)
        cv2.rectangle(image, (60, 60), (100, 100), (255, 0, 255), -1)
        return image

    def test_dim_green_with_color_shift(self):
        image = self.scene()
        # Hue outside the original [40,80] threshold; faint and compressed.
        color = cv2.cvtColor(np.uint8([[[86, 40, 45]]]), cv2.COLOR_HSV2BGR)[0, 0]
        image[63:66, 79:82] = color
        position, angle = MapProcessor()._find_robot(image)
        self.assertEqual(position, (80, 80))
        self.assertAlmostEqual(angle, math.pi / 2, places=2)

    def test_distant_green_is_not_robot_heading(self):
        image = self.scene()
        cv2.circle(image, (140, 20), 12, (0, 255, 0), -1)
        with self.assertRaises(ValueError):
            MapProcessor()._find_robot(image)

    def test_local_marker_wins_over_large_distant_green(self):
        image = self.scene()
        cv2.circle(image, (80, 65), 2, (0, 255, 0), -1)
        cv2.circle(image, (140, 20), 12, (0, 255, 0), -1)
        _, angle = MapProcessor()._find_robot(image)
        self.assertAlmostEqual(angle, math.pi / 2, places=2)
