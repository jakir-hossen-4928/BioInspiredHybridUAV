"""Regression checks for console status; pure Python, no real Webots required."""
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'controllers' / 'drone'))
import console_telemetry as ct


class FakeSensor:
    def __init__(self, values):
        self.values = values
    def getValues(self):
        return self.values
    def getRollPitchYaw(self):
        return self.values
    def getValue(self):
        return self.values


class FakeWriter:
    connected = True
    sent = 12
    dropped = 2


class FakeStreamer:
    writer = FakeWriter()


class FakeRobot:
    def getTime(self):
        return 2.5


class FakeMission:
    def __init__(self, folder):
        self.robot = FakeRobot()
        self.sensor = {
            'gps': FakeSensor((1, 2, 0.6)),
            'inertial unit': FakeSensor((0, 0.2, 0.3)),
            'gyro': FakeSensor((0.01, 0.02, 0.03)),
            'accelerometer': FakeSensor((0, 0, 9.81)),
            'ultrasonic_front': FakeSensor(float('nan')),
            'altimeter': FakeSensor(.6),
        }
        self.state = 'ROLLING'
        self.position = [1, 2, .6]
        self.lidar_count = 222
        self.last_lidar = float('inf')
        self.terrain_assessment = {'status': 'SAFE', 'slope_deg': 1.2,
                                   'roughness_m': .012, 'points': [1, 2, 3]}
        self.walk_path = [1, 2, 3]
        self.waypoints = [1, 2]
        self.walk_forces = {}
        self.wheel_angles = {'fl': .1, 'fr': .2, 'rl': .2, 'rr': .1}
        self.ground_distance = 12.5
        self.livo_stream = FakeStreamer()
        self.paused = False
        self.settings = {'console_live_enabled': True}
        self.captures_dir = Path(folder)
        self.session = 'demo'
    def foot_status(self):
        return {'estimated_support_n': {'fl': 9.5, 'fr': 9.5, 'rl': 9.5, 'rr': 9.5}}


class ConsoleTests(unittest.TestCase):
    def test_all_sections_simulated_and_estimator_not_faked(self):
        with tempfile.TemporaryDirectory() as t:
            s = ct.get_snapshot(FakeMission(t))
            result = ct.render(s)
            self.assertTrue(all(line.startswith('[SIMULATED]') for line in result.splitlines()))
            for section in ('FAST-LIVO2', 'IMU', 'GPS', 'LIDAR', 'WHEEL',
                            'FOOT FORCE', 'ROS 2', 'ASTAR'):
                self.assertIn(section, result)
            self.assertIn('NOT VERIFIED', result)
            self.assertEqual(s['model_support_estimate_total_n'], 38.0)
            self.assertIsNone(s['fast_livo2_map_and_trajectory'])
            self.assertNotIn('points', s['terrain'])

    def test_infinite_sensor_returns_do_not_corrupt_json(self):
        with tempfile.TemporaryDirectory() as t:
            m = FakeMission(t)
            ct.emit(m)
            path = Path(t) / 'demo' / 'CONSOLE_TELEMETRY.jsonl'
            record = json.loads(path.read_text().strip())
            self.assertIsNone(record['lidar_nearest_m'])
            self.assertIsNone(record['ultrasonic_front_raw'])
            self.assertTrue((Path(t) / 'demo' / 'CONSOLE_LIVE_STATUS.txt').exists())

    def test_missing_device_renders_na(self):
        with tempfile.TemporaryDirectory() as t:
            m = FakeMission(t)
            m.sensor = {}
            self.assertIn('N/A', ct.render(ct.get_snapshot(m)))

    def test_disabled_console_writes_nothing(self):
        with tempfile.TemporaryDirectory() as t:
            m = FakeMission(t)
            m.settings['console_live_enabled'] = False
            self.assertIsNone(ct.emit(m))
            self.assertFalse((Path(t) / 'demo').exists())


if __name__ == '__main__':
    unittest.main()
