"""Run the vision and planning pipeline without ROS or CoppeliaSim."""
import argparse
from pathlib import Path
import cv2
from .map_processor import MapProcessor
from .path_planner import PathPlanner


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('--output', type=Path, default=Path('outputs/path.jpg'))
    parser.add_argument('--margin', type=int, default=10)
    args = parser.parse_args()
    image = cv2.imread(str(args.image))
    if image is None:
        parser.error(f'Cannot read image: {args.image}')
    if args.margin < 0:
        parser.error('margin must be nonnegative')
    try:
        processor = MapProcessor(safety_margin=args.margin)
        grid, robot, angle, goal, scale = processor.process_image(image)
        planner = PathPlanner()
        path = planner.a_star(grid, robot, goal)
        if path is None:
            parser.exit(1, 'No collision-free path found\n')
        waypoints = planner.simplify_path(grid, path)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        overlay = processor.visualize(image, grid, robot, goal, waypoints)
        if not cv2.imwrite(str(args.output), overlay):
            parser.exit(1, f'Cannot write {args.output}\n')
        print(f'Robot: {robot}; goal: {goal}; path: {len(path)} points; '
              f'waypoints: {len(waypoints)}; output: {args.output}')
    except (ValueError, cv2.error) as exc:
        parser.exit(1, f'{exc}\n')


if __name__ == '__main__':
    main()
