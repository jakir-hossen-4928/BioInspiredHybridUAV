"""Offline wheel-legs tests. These cannot replace Webots physics/GUI tests."""
import csv
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
CONTROLLERS = ROOT / 'controllers'
sys.path.insert(0, str(CONTROLLERS / 'drone'))


class WheelWorldTests(unittest.TestCase):
    def test_both_worlds_keep_wheel_legs_and_ros_sensors(self):
        for filename in ('hybrid_wheels.wbt', 'hybrid_wheels_ros2.wbt'):
            with self.subTest(filename=filename):
                text = (ROOT / 'worlds' / filename).read_text(encoding='utf8')
                self.assertIn('controller "wheels"', text)
                self.assertEqual(text.count('DEF WHEEL_'), 4)
                self.assertEqual(text.count('type "force-3d"'), 4)
                self.assertIn('name "lidar"', text)
                self.assertIn('name "slam_camera"', text)
                self.assertIn('name "oak_depth"', text)
                self.assertIn('name "oak_rgb"', text)
                self.assertIn('name "survey_camera"', text)
                self.assertIn('width 3840 height 2160', text)
                for leg in ('fl', 'fr', 'rl', 'rr'):
                    knee = text.index('DEF LEG_' + leg.upper() + '_KNEE Transform')
                    wheel = text.index('DEF WHEEL_' + leg.upper() + ' Transform')
                    force = text.index('name "walk_force_' + leg + '"')
                    self.assertLess(knee, wheel)
                    self.assertLess(wheel, force)
                    self.assertLess(force - wheel, 2200)
                    self.assertIn('radius 0.125', text[wheel:force])
                self.assertNotIn('HingeJoint {', text)

    def test_source_generator_reproducible(self):
        spec = importlib.util.spec_from_file_location(
            '_wheel_generator', ROOT / 'tools' / 'add_wheels_worlds.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as temp:
            module.WORLDS = Path(temp)
            for name in ('hybrid.wbt', 'hybrid_ros2.wbt'):
                (module.WORLDS / name).write_bytes((ROOT / 'worlds' / name).read_bytes())
            for generated in module.build():
                self.assertEqual((ROOT / 'worlds' / generated.name).read_bytes(),
                                 generated.read_bytes())

    def test_ride_height_geometric_clearance(self):
        data = json.loads((CONTROLLERS / 'drone/settings.json').read_text())
        # Unrotated lower leg wheel contact: center at -0.02 -0.295 -0.14.
        bottom = data['wheel_body_height_m'] - 0.02 - 0.295 - 0.140 - data['wheel_radius_m']
        self.assertGreaterEqual(bottom, 0)
        self.assertLess(bottom, .05)


class WheelControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = ROOT / 'tests/test_navigation.py'
        spec = importlib.util.spec_from_file_location('_wheel_stubs', path)
        fake = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(fake)

        class WheelFake(fake.FakeSupervisor):
            def __init__(self):
                super().__init__()
                for leg in ('FL', 'FR', 'RL', 'RR'):
                    self.defs['LEG_' + leg + '_HIP'] = fake.FakeNode()
                    self.defs['LEG_' + leg + '_KNEE'] = fake.FakeNode()
                    self.defs['WHEEL_' + leg] = fake.FakeNode()

        sys.modules['controller'] = types.SimpleNamespace(Supervisor=WheelFake)
        for name in ('drone', 'hybrid', 'wheels'):
            sys.modules.pop(name, None)
        sys.path.insert(0, str(CONTROLLERS / 'hybrid'))
        spec = importlib.util.spec_from_file_location(
            '_wheels_controller', CONTROLLERS / 'wheels/wheels.py')
        wheelmod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(wheelmod)
        cls.wheelmod = wheelmod

    def test_auto_roll_hold_takeoff_and_animated_wheels(self):
        obj = self.wheelmod.WheelMission()
        self.assertEqual(obj.state, 'TO_ROLL')
        self.assertEqual(len(obj.wheel_fields), 4)
        with tempfile.TemporaryDirectory() as temp:
            obj.captures_dir = Path(temp)
            states = []
            for _ in range(2400):
                obj.robot.step(obj.step_ms)
                obj.move_step()
                obj.update_rotors()
                states.append(obj.state)
                if obj.state == 'FLYING':
                    break
            self.assertIn('ROLLING', states)
            self.assertIn('ROLL_HOLD', states)
            self.assertIn('TO_FLIGHT', states)
            self.assertEqual(obj.state, 'FLYING')
            endpoint = obj.walk_path[-1]
            self.assertAlmostEqual(obj.position[0], endpoint.x)
            self.assertAlmostEqual(obj.position[1], endpoint.y)
            self.assertAlmostEqual(obj.yaw, endpoint.yaw)
            self.assertGreater(obj.ground_distance, 1.0)
            self.assertTrue(any(abs(v) > .1 for v in obj.wheel_angles.values()))
            self.assertGreater(obj.photo_count, 0)
            with (obj.captures_dir / obj.session / 'photo_manifest.csv').open(newline='') as file:
                inspection = next(csv.DictReader(file))
            self.assertEqual(inspection['stop_id'], 'GROUND_INSPECTION')

    def test_return_home_from_ground_takes_off_before_moving_and_lands_at_ride_height(self):
        obj = self.wheelmod.WheelMission()
        obj.begin_roll()
        start = obj.planar()
        obj.return_home()
        self.assertEqual(obj.state, 'TAKEOFF')
        obj.move_step()
        self.assertEqual(obj.planar(), start)
        obj.state = 'LANDING'
        obj.position[2] = 0.61
        obj.move_step()
        self.assertEqual(obj.state, 'LANDED')
        self.assertAlmostEqual(obj.position[2], 0.60)

    def test_client_rolls_and_parks_after_touchdown(self):
        with tempfile.TemporaryDirectory() as temp, patch.dict(os.environ, UAV_CLIENT_EVIDENCE=temp):
            obj = self.wheelmod.WheelMission()
            obj.position[2] = obj.rolling_height + .01
            obj.state = 'LANDING'
            obj.returning = True
            obj.last_print = math.inf
            trajectory = []
            obj.livo_stream = types.SimpleNamespace(
                update=lambda t, xyz, yaw: trajectory.append(tuple(xyz)), close=lambda: None)
            obj.evaluate_terrain = lambda: None
            obj.write_sensor_dashboard = lambda **kwargs: None
            obj.robot.exportImage = lambda *args: None
            obj.robot.simulationQuit = lambda code: None
            geometry = types.SimpleNamespace(
                getBaseTypeName=lambda: 'Box',
                getField=lambda name: types.SimpleNamespace(getSFVec3f=lambda: (2., 2., 2.)),
            )
            obstacle = types.SimpleNamespace(
                getBaseTypeName=lambda: 'Solid',
                getPosition=lambda: (-41., -47., 1.),
                getOrientation=lambda: (1, 0, 0, 0, 1, 0, 0, 0, 1),
                getField=lambda name: types.SimpleNamespace(getSFNode=lambda: geometry),
            )
            children = obj.robot.getRoot().getField('children')
            children.getCount = lambda: int(obj.ground_distance >= 2.)
            children.getMFNode = lambda index: obstacle
            obj.run()
            result = json.loads((Path(temp) / 'farm-result.json').read_text())
        self.assertEqual(result['state'], 'LANDED')
        self.assertGreater(result['post_landing_roll_m'], 1.)
        self.assertNotEqual(result['landing_position_m'], result['final_position_m'])
        self.assertNotIn('TO_FLIGHT', result['states'])
        self.assertGreater(result['final_position_m'][0], -32.)
        gap = min(math.hypot(max(0., abs(x + 41.) - 1.), max(0., abs(y + 47.) - 1.))
                  for x, y, _ in trajectory)
        self.assertGreater(gap, .72)

    def test_mode_switch_preserves_screened_descent(self):
        obj = self.wheelmod.WheelMission()
        obj.begin_roll()
        obj.state = 'FLYING'
        obj.position = [-49, -47, 6.0]
        obj.evaluate_terrain = lambda force=False: {'status': 'SAFE'}
        self.assertTrue(obj.request_walk())
        pending = obj.pending_hybrid_route
        self.assertTrue(obj.request_roll())
        self.assertEqual(obj.state, 'TO_ROLL')
        self.assertIs(obj.pending_hybrid_route, pending)
        self.assertTrue(obj.request_walk())
        self.assertEqual(obj.state, 'TO_WALK')
        self.assertIs(obj.pending_hybrid_route, pending)


    def test_roll_to_walk_to_fly(self):
        obj = self.wheelmod.WheelMission()
        obj.begin_roll()
        self.assertEqual(obj.state, 'ROLLING')
        self.assertTrue(obj.request_walk())
        self.assertEqual(obj.state, 'WALKING')
        self.assertTrue(obj.request_roll())
        self.assertEqual(obj.state, 'ROLLING')
        self.assertTrue(obj.request_flight())
        self.assertEqual(obj.state, 'TO_FLIGHT')

    def test_flight_to_roll_rejects_unsafe_terrain(self):
        obj = self.wheelmod.WheelMission()
        obj.state = 'FLYING'
        obj.position = [-49, -47, 6.]
        obj.evaluate_terrain = lambda force=False: {'status': 'UNSAFE', 'reason': 'rough'}
        self.assertFalse(obj.request_roll())
        self.assertEqual(obj.state, 'FLYING')
        obj.evaluate_terrain = lambda force=False: {'status': 'SAFE', 'reason': 'flat'}
        self.assertTrue(obj.request_roll())
        self.assertEqual(obj.state, 'TO_ROLL')

    def test_wheel_rotation_is_differential_for_turns(self):
        obj = self.wheelmod.WheelMission()
        obj._advance_wheels(.5, .1)
        self.assertNotAlmostEqual(obj.wheel_angles['fl'], obj.wheel_angles['fr'])
        self.assertAlmostEqual(obj.wheel_angles['fl'], obj.wheel_angles['rl'])
        self.assertAlmostEqual(obj.wheel_angles['fr'], obj.wheel_angles['rr'])

    def test_sensor_replans_while_rolling(self):
        obj = self.wheelmod.WheelMission()
        obj.begin_roll()
        obj.sensor['ultrasonic_front'].getValue = lambda: 2.5
        obj.robot.time = 5.0
        obj.sense_obstacles()
        self.assertTrue(obj.sensor_obstacle is not None or obj.warning_message)

    def test_flight_depth_hit_replans_around_an_unmapped_box(self):
        obj = self.wheelmod.WheelMission()
        obj.position = [0., 0., 13.]
        obj.state = 'FLYING'
        obj.goal = (13., 0.)
        obj.obstacles = []
        obj.draw_paths = lambda **kwargs: None
        self.assertTrue(obj.replan())
        obj.stereo = [None, None, None]
        obj.oak_depth = types.SimpleNamespace(
            getWidth=lambda: 5, getHeight=lambda: 3, getFov=lambda: 1.4,
            getRangeImage=lambda: [math.inf] * 5 + [math.inf, 6., 6., 6., math.inf] + [math.inf] * 5,
        )
        obj.sense_obstacles()
        self.assertEqual(len(obj.obstacles), 1)
        self.assertEqual(obj.obstacles[0]['name'], 'OAK depth')
        self.assertEqual(obj.waypoints[-1], obj.goal)
        self.assertGreater(len(obj.waypoints), 2)
        self.assertFalse(obj.planner.clear_segment(obj.planar(), obj.goal))

    def test_telemetry_says_roll_and_keeps_force_limits(self):
        obj = self.wheelmod.WheelMission()
        obj.begin_roll()
        with tempfile.TemporaryDirectory() as temp:
            obj.captures_dir = Path(temp)
            obj.write_hybrid_status()
            status = json.loads((Path(temp) / obj.session / 'hybrid_status.json').read_text())
        self.assertEqual(status['mode'], 'ROLL')
        self.assertIn('motion_model', status['wheels'])
        self.assertIn('not a sensor reading', status['feet']['note'])


if __name__ == '__main__':
    unittest.main()
