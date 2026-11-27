"""Compare the three ground planners on the real scene map and write a CSV.

    python Drone/tools/compare_routes.py

Prints path length, node count and peak curvature so the report can state the
difference between the grid A*, the 8-heading mode-augmented planner and the
continuous-curvature Hybrid A*.
"""
import csv
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DRONE = HERE.parent
sys.path.insert(0, str(DRONE / 'controllers' / 'drone'))
from planner import Planner
from hybrid_planner import HybridStatePlanner
from hybrid_astar import plan as hybrid_astar


def length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def peak_curvature(poses):
    worst = 0.0
    for a, b in zip(poses, poses[1:]):
        span = math.hypot(b.x - a.x, b.y - a.y)
        if span < 1e-6:
            continue
        yaw_a = getattr(a, 'yaw', getattr(a, 'yaw_rad', 0.0))
        yaw_b = getattr(b, 'yaw', getattr(b, 'yaw_rad', 0.0))
        turn = abs((yaw_b - yaw_a + math.pi) % (2 * math.pi) - math.pi)
        worst = max(worst, turn / span)
    return worst


def main():
    scene = json.loads((DRONE / 'controllers' / 'drone' / 'scene.json').read_text())
    grid = Planner(scene['obstacles'], resolution=.75, boundary=scene['bounds_m'], safety=1.2)
    start = tuple(scene['start'])
    goal = (-39.0, -36.0)
    rows = []

    route = grid.route(start, goal)
    rows.append(('grid A* (8 directions)', len(route), length(route), float('inf')))

    for direction in (8,):
        states = HybridStatePlanner(grid).plan(start, goal, 'WALK', 'WALK', 0.0)
        points = [(s.x, s.y) for s in states]
        rows.append(('mode-augmented %d-heading' % direction, len(states),
                     length(points), peak_curvature(states)))

    poses = hybrid_astar(grid, start, goal, start_yaw=0.0, goal_yaw=0.0, curvature=0.55)
    rows.append(('Hybrid A* (continuous)', len(poses), length([(p.x, p.y) for p in poses]),
                 peak_curvature(poses)))

    out = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / 'planner_comparison.csv'
    with out.open('w', encoding='utf-8', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['planner', 'nodes', 'length_m', 'peak_curvature_1_per_m'])
        writer.writerows(rows)
    for name, nodes, dist, curvature in rows:
        shown = 'n/a' if math.isinf(curvature) else '%.3f' % curvature
        print('%-28s nodes=%-4d length=%6.2f m  peak_curvature=%s' % (name, nodes, dist, shown))
    print('written:', out)


if __name__ == '__main__':
    main()
