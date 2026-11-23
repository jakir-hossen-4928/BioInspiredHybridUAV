"""Drone known-map manual route/driver tests without requiring Webots GUI."""
import copy
import importlib.util
import json
import math
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
CONTROLLER = ROOT/'controllers'/'drone'
sys.path.insert(0, str(CONTROLLER))
from manual_routes import validate_manual_payload, check_safe_route
from test_navigation import FakeSupervisor


class ManualRouteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene=json.loads((CONTROLLER/'scene.json').read_text())
        cls.payload=json.loads((CONTROLLER/'manual_mission.json').read_text())
        sys.modules['controller']=types.SimpleNamespace(Supervisor=FakeSupervisor)
        spec=importlib.util.spec_from_file_location('_manual_controller',CONTROLLER/'drone.py')
        cls.mod=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.mod)

    def test_sample_manual_route_valid_and_returns_home(self):
        stops,config=validate_manual_payload(self.payload,self.scene)
        self.assertEqual(len(stops),5)
        legs,dist=check_safe_route(stops,self.scene,config)
        self.assertEqual(len(legs),6)
        self.assertGreater(dist,300)
        self.assertEqual(legs[-1][-1],tuple(self.scene['start']))

    def test_obstacle_boundary_and_invalid_data_are_rejected(self):
        for point in ([500,500],[-10.5,3.0]):
            with self.subTest(point=point):
                item=copy.deepcopy(self.payload)
                item['stops'][0]['point']=point
                try:
                    stops,cfg=validate_manual_payload(item,self.scene)
                except ValueError:
                    continue
                with self.assertRaises(ValueError):
                    check_safe_route(stops,self.scene,cfg)
        for change in ({'hover_s':50},{'take_photo':'yes'},{'point':['oops',4]}, {'id':'A B'}):
            item=copy.deepcopy(self.payload)
            item['stops'][0].update(change)
            with self.assertRaises(ValueError):
                validate_manual_payload(item,self.scene)

    def test_hot_reload_changes_mission_without_restart(self):
        m=self.mod.Mission()
        self.assertEqual(m.mode_name,'FULL_FARM_SURVEY')
        self.assertTrue(m.reload_manual_route())
        self.assertEqual(m.mode_name,'MANUAL_PATH')
        self.assertEqual(len(m.route_stops),5)
        self.assertEqual(m.state,'TAKEOFF')
        self.assertEqual(m.goal,[-39.,-36.])
        m.robot.step(m.step_ms)
        m.move_step()
        self.assertGreater(m.position[2],.585)
        self.assertEqual(len(m.rotor_visuals),4)
        self.assertTrue(m.waypoints)

    def test_u_keyboard_key_activates_manual_mode(self):
        m=self.mod.Mission()
        keys=[ord('u'),-1]
        m.keyboard.getKey=lambda: keys.pop(0)
        m.read_keys()
        self.assertEqual(m.mode_name,'MANUAL_PATH')
        self.assertEqual(m.stop_index,0)
        self.assertTrue(m.waypoints)
        self.assertEqual(len(m.manual_marker_defs),5)

    def test_custom_hover_without_photos(self):
        fake=copy.deepcopy(self.payload)
        fake['stops']=[{'id':'CUSTOM_01','name':'Inspect Only',
                        'point':[-39.,-36.], 'hover_s':1.4,'take_photo':False}]
        fake['flight_settings']={'flight_speed_mps':5.0,
                                 'target_altitude_m':9.5,'obstacle_clearance_m':1.0}
        original=self.mod.load_json
        def customized(filename):
            return fake if filename=='manual_mission.json' else original(filename)
        with patch.object(self.mod,'load_json',side_effect=customized):
            m=self.mod.Mission()
            self.assertTrue(m.reload_manual_route())
        self.assertEqual(m.speed,5.)
        self.assertEqual(m.altitude,9.5)
        self.assertEqual(m.stop_hover_seconds(),1.4)
        with tempfile.TemporaryDirectory() as temp:
            m.captures_dir=Path(temp)
            for i in range(10000):
                m.robot.step(m.step_ms)
                m.move_step()
                if m.state=='LANDED':break
        self.assertEqual(m.state,'LANDED')
        self.assertEqual(m.photo_count,0)
        self.assertTrue(m.returning)

    def test_manual_two_photo_stops_saves_only_opted_in_stop(self):
        fake=copy.deepcopy(self.payload)
        fake['stops']=[{'id':'CUSTOM_01','name':'No photo', 'point':[-39.,-36.],
                        'hover_s':1.5,'take_photo':False},
                       {'id':'CUSTOM_02','name':'Photo station','point':[-28.,-36.],
                        'hover_s':1.6,'take_photo':True}]
        original=self.mod.load_json
        with patch.object(self.mod,'load_json',side_effect=lambda fname:
                          fake if fname=='manual_mission.json' else original(fname)):
            m=self.mod.Mission()
            self.assertTrue(m.reload_manual_route())
        with tempfile.TemporaryDirectory() as temp:
            m.captures_dir=Path(temp)
            for _ in range(13000):
                m.robot.step(m.step_ms)
                m.move_step()
                if m.state=='LANDED':break
            self.assertEqual(m.state,'LANDED')
            self.assertEqual(m.photo_count,2)
            self.assertEqual(len(list(Path(temp).rglob('*.png'))),2)
            self.assertTrue(list(Path(temp).rglob('MISSION_GALLERY.html')))

    def test_invalid_new_json_does_not_replace_active_path(self):
        m=self.mod.Mission()
        self.assertTrue(m.reload_manual_route())
        original_stops=copy.deepcopy(m.manual_stops)
        bad=copy.deepcopy(self.payload)
        bad['stops'][0]['point']=[-10.5,3.]  # inside barn
        original=self.mod.load_json
        with patch.object(self.mod,'load_json',side_effect=lambda fname:
                          bad if fname=='manual_mission.json' else original(fname)):
            self.assertFalse(m.reload_manual_route())
        self.assertEqual(m.manual_stops,original_stops)
        self.assertEqual(m.mode_name,'MANUAL_PATH')


if __name__=='__main__':unittest.main()
