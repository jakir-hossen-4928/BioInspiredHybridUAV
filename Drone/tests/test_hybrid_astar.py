"""Offline checks for the continuous-curvature Hybrid A* route planner."""
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'controllers' / 'drone'))
from planner import Planner
from hybrid_astar import Pose, plan, arc_poses


class HybridAStarTests(unittest.TestCase):
    def grid(self):
        obstacles = [
            {'name': 'barn', 'x': 0.0, 'y': 0.0, 'sx': 6.0, 'sy': 4.0},
            {'name': 'shed', 'x': 6.0, 'y': -6.0, 'sx': 3.0, 'sy': 3.0},
        ]
        return Planner(obstacles, resolution=.75, boundary=14.0, safety=0.6)

    def test_route_is_drivable_and_clear(self):
        grid = self.grid()
        route = plan(grid, (-9.0, -9.0), (9.0, 9.0), start_yaw=0.0, curvature=0.55)
        self.assertGreater(len(route), 3)
        self.assertLess(math.dist((route[0].x, route[0].y), (-9.0, -9.0)), 1e-6)
        self.assertLess(math.dist((route[-1].x, route[-1].y), (9.0, 9.0)), 1.0)
        for pose in route:
            self.assertTrue(grid.is_free(grid.xy_to_cell(pose.x, pose.y)), pose)
        for a, b in zip(route, route[1:]):
            self.assertLessEqual(abs(b.curvature), 0.55)
            samples = arc_poses(a, b.curvature, b.length, segments=4)
            self.assertEqual(samples[-1], Pose(b.x, b.y, b.yaw))
            for pose in samples:
                self.assertTrue(grid.is_free(grid.xy_to_cell(pose.x, pose.y)), pose)

    def test_goal_yaw_is_reached(self):
        grid = self.grid()
        route = plan(grid, (-9.0, 9.0), (9.0, -9.0), start_yaw=math.pi / 2,
                     goal_yaw=-math.pi / 2, curvature=0.5)
        error = (route[-1].yaw + math.pi / 2 + math.pi) % (2 * math.pi) - math.pi
        self.assertLessEqual(abs(error), 0.4)

    def test_blocked_goal_is_rejected(self):
        grid = self.grid()
        with self.assertRaises(ValueError):
            plan(grid, (-9.0, -9.0), (0.0, 0.0))

    def test_arc_geometry_matches_curvature(self):
        start = Pose(0.0, 0.0, 0.0)
        track = arc_poses(start, 0.5, 2.0, segments=8)
        end = track[-1]
        self.assertAlmostEqual(end.yaw, 1.0, places=6)
        # A 2 m arc at curvature 0.5 has radius 2 m and chord 2*R*sin(L/2R).
        chord = 2 * 2.0 * math.sin(2.0 / (2 * 2.0))
        self.assertAlmostEqual(math.hypot(end.x, end.y), chord, places=6)


if __name__ == '__main__':
    unittest.main()
