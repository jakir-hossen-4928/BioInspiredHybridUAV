"""Regression checks for moving obstacle occlusion and a blocked goal."""
import sys
import unittest
from pathlib import Path

DRONE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(DRONE / 'controllers/drone'),
                str(DRONE / 'validation/controllers/inertial_flight')]
from navigation_trial import ObstacleTracks, braking_segment
from planner import Planner


class FlightSafetyTests(unittest.TestCase):
    def test_obstacle_reversal_remains_occupied_after_camera_dropout(self):
        tracks = ObstacleTracks()
        for t, y in enumerate((-.75, -.1, .75)):
            tracks.observe([dict(x=8., y=y, sx=1., sy=1.2)], t / 2)
        tracks.observe([], 5.)
        grid = Planner(tracks.footprints(5., .5, .75), .25, 14., .85)
        self.assertFalse(grid.clear_segment((6., -1.15), (10., -1.15)))

    def test_destination_obstacle_stays_occupied(self):
        tracks = ObstacleTracks()
        tracks.observe([dict(x=10.5, y=0., sx=1., sy=1.2)], 1.)
        grid = Planner(tracks.footprints(2., .5, .75), .25, 14., .85)
        with self.assertRaisesRegex(ValueError, 'Goal lies inside'):
            grid.route((5., 0.), (10.5, 0.))

    def test_brake_includes_perception_and_odometry_age(self):
        old = braking_segment((0., 0.), (.75, 0.), .5, 2.4)
        fresh = braking_segment((0., 0.), (.75, 0.), .2, 2.4)
        self.assertAlmostEqual(old[0] - fresh[0], .225)
