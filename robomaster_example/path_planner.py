"""Grid planning with conservative corner checks and collision-checked shortcuts."""
import heapq
import math


class PathPlanner:
    def _is_valid(self, grid, point):
        x, y = point
        return 0 <= y < grid.shape[0] and 0 <= x < grid.shape[1] and grid[y, x] == 1

    def a_star(self, grid_map, start, goal):
        start, goal = tuple(map(int, start)), tuple(map(int, goal))
        if not self._is_valid(grid_map, start) or not self._is_valid(grid_map, goal):
            return None
        costs, parents = {start: 0.0}, {}
        queue = [(math.dist(start, goal), 0.0, start)]
        while queue:
            _, cost, current = heapq.heappop(queue)
            if cost > costs[current]:
                continue
            if current == goal:
                path = [current]
                while current in parents:
                    current = parents[current]
                    path.append(current)
                return path[::-1]
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = current[0] + dx, current[1] + dy
                if not self._is_valid(grid_map, nxt):
                    continue
                if dx and dy and not (
                    self._is_valid(grid_map, (current[0] + dx, current[1])) and
                    self._is_valid(grid_map, (current[0], current[1] + dy))):
                    continue
                candidate = cost + math.hypot(dx, dy)
                if candidate < costs.get(nxt, math.inf):
                    costs[nxt], parents[nxt] = candidate, current
                    heapq.heappush(queue, (candidate + math.dist(nxt, goal), candidate, nxt))
        return None

    def _is_line_free(self, grid_map, start, end):
        # Supercover: check all cells touched by the continuous segment, including
        # both side cells at a diagonal corner crossing.
        x, y = map(int, start)
        ex, ey = map(int, end)
        nx, ny = abs(ex - x), abs(ey - y)
        sx, sy = (1 if ex > x else -1), (1 if ey > y else -1)
        ix = iy = 0
        if not self._is_valid(grid_map, (x, y)):
            return False
        while ix < nx or iy < ny:
            decision = (1 + 2 * ix) * ny - (1 + 2 * iy) * nx
            if decision == 0:
                if not (self._is_valid(grid_map, (x + sx, y)) and
                        self._is_valid(grid_map, (x, y + sy))):
                    return False
                x, y, ix, iy = x + sx, y + sy, ix + 1, iy + 1
            elif decision < 0:
                x, ix = x + sx, ix + 1
            else:
                y, iy = y + sy, iy + 1
            if not self._is_valid(grid_map, (x, y)):
                return False
        return True

    def simplify_path(self, grid_map, path, max_points=20, min_distance=6):
        # Never merge or downsample unchecked segments to meet a waypoint budget.
        if not path:
            return []
        simplified, index = [path[0]], 0
        while index < len(path) - 1:
            last = index + 1
            for candidate in range(index + 1, len(path)):
                if not self._is_line_free(grid_map, path[index], path[candidate]):
                    break
                last = candidate
            if not self._is_line_free(grid_map, path[index], path[last]):
                raise ValueError('Input path contains an obstructed segment')
            simplified.append(path[last])
            index = last
        return simplified
