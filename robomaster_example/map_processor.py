"""HSV marker localization and footprint-inflated occupancy mapping."""
import cv2
import numpy as np
import math

class MapProcessor:
    def __init__(self, safety_margin=6):
        self.safety_margin = safety_margin
        self.scale = None
        self.robot_radius = None

    def process_image(self, image):

        if image is None or image.ndim != 3 or image.shape[2] not in (3, 4):
            raise ValueError('Expected a BGR or BGRA image')
        if image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        robot_position, robot_angle = self._find_robot(image)
        goal_position = self._find_goal(image)

        if robot_position is None or robot_angle is None or goal_position is None:
            raise ValueError('Robot markers or goal were not detected')

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        kernel = np.ones((5, 5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        grid_map = np.where(binary > 0, 1, 0)

        robot_area = cv2.dilate(self.robot_mask, kernel, iterations=1)
        grid_map[robot_area > 0] = 1

        goal_area = cv2.dilate(self.goal_mask, kernel, iterations=1)
        grid_map[goal_area > 0] = 1

        if self.scale is None:
            points = self.robot_contour
            (_, _), (width, height), _ = cv2.minAreaRect(points)
            side = max(width, height)
            if side <= 0:
                raise ValueError('Robot marker is too small to calibrate')
            self.scale = [0.55 / side, 0.55 / side]
            self.robot_radius = int(math.ceil(math.hypot(width, height) / 2))
        radius = self.robot_radius + self.safety_margin
        inflation = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))
        obstacles = cv2.dilate((grid_map == 0).astype(np.uint8), inflation)
        grid_map = (obstacles == 0).astype(np.uint8)
        scale = self.scale


        return grid_map, robot_position, robot_angle, goal_position, scale

    def _find_robot(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_purple = np.array([120, 50, 50])
        upper_purple = np.array([150, 255, 255])

        purple_mask = cv2.inRange(hsv, lower_purple, upper_purple)

        contours, _ = cv2.findContours(purple_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.robot_mask = np.zeros_like(purple_mask)

        if not contours:
            raise ValueError('Purple robot marker was not detected')
        largest_contour = max(contours, key=cv2.contourArea)

        self.robot_contour = largest_contour
        x, y, w, h = cv2.boundingRect(largest_contour)

        self.robot_mask[y:y+h, x:x+w] = 255

        px = x + w // 2
        py = y + h // 2
        robot_position = (px, py)

        green_mask = cv2.inRange(hsv, np.array([32, 25, 20]), np.array([98, 255, 255]))
        yy, xx = np.ogrid[:image.shape[0], :image.shape[1]]
        distance = (xx - px) ** 2 + (yy - py) ** 2
        near = (distance <= (max(w, h) * 0.9) ** 2) & (distance >= 3 ** 2)
        green_mask[~near] = 0
        count, labels, stats, centers = cv2.connectedComponentsWithStats(green_mask)
        if count <= 1:
            raise ValueError('Green heading marker was not detected near the robot')
        component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        gx, gy = centers[component]

        dx = gx - px  # x difference between green marker and robot center
        dy = py - gy  # y difference (note: y is inverted in image coordinates)
        robot_angle = math.atan2(dy, dx)  # in radians

        return robot_position, robot_angle


    def _find_goal(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        self.goal_mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(self.goal_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)

            x, y, w, h = cv2.boundingRect(largest_contour)

            self.goal_mask[y:y+h, x:x+w] = 255

            cx = x + w // 2
            cy = y + h // 2
            return (cx, cy)

        raise ValueError('Red goal marker was not detected')


    def visualize(self, image, grid_map, robot_pos, goal_pos, path=None):
        vis = image.copy()

        vis[grid_map == 0] = (0, 0, 255)

        cv2.circle(vis, robot_pos, 5, (255, 0, 0), -1)
        cv2.putText(vis, "Robot", (robot_pos[0] + 15, robot_pos[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        cv2.circle(vis, goal_pos, 5, (0, 255, 0), -1)
        cv2.putText(vis, "Goal", (goal_pos[0] + 15, goal_pos[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if path:
            for i in range(len(path) - 1):
                cv2.line(vis, path[i], path[i+1], (0, 255, 255), 2)

            for point in path:
                cv2.circle(vis, point, 4, (0, 255, 0), -1)

        return vis
