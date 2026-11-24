"""Offline checks for the simulated LiDAR-inertial estimator maths."""
import math
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'integration' / 'ros2'))
import fast_livo2_sim as sim


def cloud():
    # Irregular spacing: a regular lattice lets nearest-neighbour ICP snap to a
    # neighbouring point, which would test the wrong thing.
    points = []
    for ix in range(-6, 7):
        for iy in range(-6, 7):
            points.append((ix * 0.37 + (ix * iy) % 7 * 0.11,
                           iy * 0.41 + (ix + iy) % 5 * 0.13,
                           (ix * iy) % 11 * 0.19))
    return points


class EstimatorTests(unittest.TestCase):
    def test_icp_recovers_known_transform(self):
        reference = cloud()
        truth = (0.5, -0.3, 0.2, 0.05)
        # The map holds world points; the scan arrives in the body frame, so
        # ICP has to recover the body-to-world transform that generated them.
        scan_map = sim.ScanMap()
        scan_map.add(reference, *truth)
        tx, ty, tz, yaw, used = sim.icp_step(reference, scan_map, 0.0, 0.0, 0.0, 0.0, iterations=12)
        self.assertEqual(used, len(reference))
        self.assertAlmostEqual(tx, truth[0], delta=0.08)
        self.assertAlmostEqual(ty, truth[1], delta=0.08)
        self.assertAlmostEqual(tz, truth[2], delta=0.08)
        self.assertAlmostEqual(yaw, truth[3], delta=0.02)

    def test_solve4_solves_linear_system(self):
        matrix = [[4.0, 1.0, 0.0, 0.0], [1.0, 3.0, 0.0, 0.0],
                  [0.0, 0.0, 2.0, 0.0], [0.0, 0.0, 0.0, 5.0]]
        solution = sim.solve4(matrix, [5.0, 4.0, 2.0, 10.0])
        self.assertAlmostEqual(solution[0], 1.0)
        self.assertAlmostEqual(solution[1], 1.0)
        self.assertAlmostEqual(solution[2], 1.0)
        self.assertAlmostEqual(solution[3], 2.0)

    def test_imu_prediction_integrates_acceleration(self):
        estimator = sim.Estimator(Path(tempfile.gettempdir()))
        for step in range(251):
            estimator.imu({'sim_time_s': step * 0.008,
                           'accel_m_s2': [2.0, 0, 0],
                           'omega_rad_s': [0, 0, 0]})
        self.assertAlmostEqual(estimator.velocity[0], 4.0, delta=0.05)
        self.assertGreater(estimator.pose[0], 0.5)
        self.assertAlmostEqual(estimator.pose[3], 0.0, places=6)

    def test_imu_prediction_integrates_yaw_rate(self):
        estimator = sim.Estimator(Path(tempfile.gettempdir()))
        for step in range(251):
            estimator.imu({'sim_time_s': step * 0.008,
                           'accel_m_s2': [0, 0, 0],
                           'omega_rad_s': [0, 0, 0.5]})
        self.assertAlmostEqual(estimator.pose[3], 1.0, delta=0.05)

    def test_report_writes_track_and_map(self):
        with tempfile.TemporaryDirectory() as temp:
            estimator = sim.Estimator(Path(temp))
            estimator.map.add([(0, 0, 0), (1, 0, 0), (0, 1, 0)], 0, 0, 0, 0)
            estimator.track = [(0.0, (0, 0, 0), (0, 0, 0)),
                               (1.0, (1, 0, 0), (1.1, 0, 0))]
            summary = estimator.report()
            self.assertAlmostEqual(summary['rmse_m'], math.sqrt(0.005))
            self.assertEqual(summary['poses'], 2)
            self.assertTrue((Path(temp) / 'livo_sim_trajectory.csv').exists())
            self.assertTrue((Path(temp) / 'livo_sim_map.ply').exists())


if __name__ == '__main__':
    unittest.main()
