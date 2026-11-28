"""Regression checks for moving obstacle occlusion and a blocked goal."""
import sys
import unittest
from pathlib import Path

DRONE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(DRONE / 'controllers/drone'),
                str(DRONE / 'validation/controllers/inertial_flight')]
from navigation_trial import ObstacleTracks, altitude_bias, braking_segment, margin_exit, view_route
from planner import Planner


class FlightSafetyTests(unittest.TestCase):
    def test_obstacle_reversal_remains_occupied_after_camera_dropout(self):
        tracks = ObstacleTracks()
        for t, y in enumerate((-.75, -.1, .75)):
            tracks.observe([dict(x=8., y=y, sx=1., sy=1.2)], t / 2)
        tracks.observe([], 5.)
        grid = Planner(tracks.footprints(.5, .75), .25, 14., .85)
        self.assertFalse(grid.clear_segment((6., -1.15), (10., -1.15)))

    def test_destination_obstacle_stays_occupied(self):
        tracks = ObstacleTracks()
        tracks.observe([dict(x=10.5, y=0., sx=1., sy=1.2)], 1.)
        grid = Planner(tracks.footprints(.5, .75), .25, 14., .85)
        with self.assertRaisesRegex(ValueError, 'Goal lies inside'):
            grid.route((5., 0.), (10.5, 0.))

    def test_lidar_occlusion_cannot_clear_a_track(self):
        tracks = ObstacleTracks()
        track = dict(x=8., y=0., sx=1., sy=1.)
        tracks.observe([track], 0.)
        background = [(12., -.3), (12., 0.), (12., .3)]
        for t in (.1, .2, .3):
            tracks.observe([], t, background + [(5., 0.)])
        self.assertEqual(len(tracks.footprints(.5, .75)), 1)
        for t in (.4, .5, .6):
            tracks.observe([], t, background)
        self.assertEqual(tracks.footprints(.5, .75), [])

    def test_brake_includes_perception_and_odometry_age(self):
        old = braking_segment((0., 0.), (.75, 0.), .5, 2.4)
        fresh = braking_segment((0., 0.), (.75, 0.), .2, 2.4)
        self.assertAlmostEqual(old[0] - fresh[0], .225)

    def test_margin_exit_only_moves_away_from_physical_occupancy(self):
        grid = Planner([dict(x=8., y=0., sx=1., sy=1.2)], .25, 14., .85)
        self.assertIsNone(margin_exit((8., .8), grid, .5))
        end = margin_exit((8., 1.3), grid, .5)
        self.assertGreater(end[1], 1.3)
        self.assertTrue(grid.is_free(grid.xy_to_cell(*end)))

    def test_blocked_goal_observation_route_preserves_occupancy(self):
        goal = (8., 0.)
        grid = Planner([dict(x=8., y=0., sx=1., sy=2.)], .25, 14., .85)
        path = view_route(grid, (5., 1.), goal)
        self.assertNotEqual(path[-1], goal)
        self.assertFalse(grid.is_free(grid.xy_to_cell(*goal)))
        self.assertTrue(all(grid.clear_segment(a, b) for a, b in zip(path, path[1:])))

    def test_altimeter_gradually_corrects_lio_height_drift(self):
        bias = 0.
        for _ in range(150):
            bias = altitude_bias(bias, -.8, 0.)
        self.assertAlmostEqual(-.8 + bias, 0., places=3)
        self.assertEqual(altitude_bias(bias, -.8, float('nan')), bias)
        self.assertAlmostEqual(altitude_bias(bias, -.8, 20.) - bias, .02)
