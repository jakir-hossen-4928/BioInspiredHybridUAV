"""Unit and mocked Webots tests. Does not verify Webots rendering or real foot forces."""
import importlib.util
import json
import math
import sys
import tempfile
import types
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
CONT=ROOT/'controllers'/'drone'
sys.path.insert(0,str(CONT))
from planner import Planner
from hybrid_planner import HybridStatePlanner, choose_safe_walk_goal
from hybrid_support import gait_angles, force_estimate, projected_obstacle


class PlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene=json.loads((CONT/'scene.json').read_text())

    def grid(self):
        return Planner(self.scene['obstacles'], resolution=.75,
                       boundary=self.scene['bounds_m'], safety=1.2)

    def test_walk_has_heading_and_avoids_obstacles(self):
        grid=self.grid(); start=tuple(self.scene['start'])
        goal,route=choose_safe_walk_goal(grid,start)
        self.assertGreater(len(route),2)
        self.assertTrue(all(n.mode=='WALK' for n in route))
        self.assertTrue(all(grid.is_free(grid.xy_to_cell(n.x,n.y)) for n in route))
        self.assertTrue(all(grid.clear_segment((a.x,a.y),(b.x,b.y))
                            for a,b in zip(route,route[1:])))
        self.assertLess(math.dist((route[-1].x,route[-1].y),goal),1.0)

    def test_augmented_search_crosses_modes_at_authorized_site(self):
        grid=self.grid(); origin=tuple(self.scene['start'])
        transition=(-43.0,-43.0);goal=(-39.0,-42.0)
        result=HybridStatePlanner(grid,transition_zones=[transition]).plan(
            origin,goal,start_mode='FLIGHT',goal_mode='WALK',start_yaw=0)
        self.assertEqual(result[0].mode,'FLIGHT')
        self.assertEqual(result[-1].mode,'WALK')
        switches=[(a,b) for a,b in zip(result,result[1:]) if a.mode!=b.mode]
        self.assertEqual(len(switches),1)
        self.assertTrue(all(p.z == (13.0 if p.mode == 'FLIGHT' else 0.6) for p in result))
        self.assertLessEqual(math.dist((result[-1].x,result[-1].y),goal),grid.resolution)
        reverse=HybridStatePlanner(grid,transition_zones=[transition]).plan(
            goal,origin,start_mode='WALK',goal_mode='FLIGHT')
        self.assertEqual((reverse[0].mode, reverse[-1].mode), ('WALK', 'FLIGHT'))
        self.assertTrue(all(grid.is_free(grid.xy_to_cell(p.x,p.y)) for p in reverse))
        a,b=switches[0]
        self.assertAlmostEqual(a.x,b.x)
        self.assertAlmostEqual(a.y,b.y)
        self.assertEqual(grid.xy_to_cell(a.x,a.y),grid.xy_to_cell(*transition))

    def test_no_transition_without_allowed_site(self):
        grid=self.grid(); origin=tuple(self.scene['start'])
        with self.assertRaises(ValueError):
            HybridStatePlanner(grid).plan(origin,(-39,-36),'FLIGHT','WALK')

    def test_gait_diagonal_pairs_and_force_estimate(self):
        gait=gait_angles(math.pi/2)
        self.assertAlmostEqual(gait['fl'],gait['rr'])
        self.assertAlmostEqual(gait['fr'],gait['rl'])
        self.assertAlmostEqual(gait['fl'],-gait['fr'])
        forces=force_estimate(3.9)
        self.assertAlmostEqual(sum(forces.values()),3.9*9.81,places=5)
        self.assertEqual(sum(force_estimate(3.9,False).values()),0)

    def test_front_range_projection(self):
        self.assertIsNone(projected_obstacle((0,0),0,float('inf')))
        self.assertIsNone(projected_obstacle((0,0),0,-1))
        d=projected_obstacle((1,2),math.pi/2,2.3)
        self.assertAlmostEqual(d.x,1)
        self.assertGreater(d.y,4.3)

    def test_world_has_articulated_legs_and_force_devices(self):
        txt=(ROOT/'worlds/hybrid.wbt').read_text()
        self.assertIn('controller "hybrid"',txt)
        self.assertEqual(txt.count('type "force-3d"'),4)
        self.assertEqual(txt.count('name "walk_force_'),4)
        for leg in ('FL','FR','RL','RR'):
            self.assertEqual(txt.count('DEF LEG_'+leg+'_HIP Transform'),1)
            self.assertEqual(txt.count('DEF LEG_'+leg+'_KNEE Transform'),1)
        # World is deliberately gravity-free: raw force vectors may be zero.
        self.assertIn('gravity 0',txt)
        self.assertIn('width 3840 height 2160',txt)
        self.assertIn('name "slam_camera"',txt)


class HybridControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fake_path=ROOT/'tests/test_navigation.py'
        spec=importlib.util.spec_from_file_location('_old_tests_for_stubs',fake_path)
        old=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(old)
        class HybridFake(old.FakeSupervisor):
            def __init__(self):
                super().__init__()
                for k in ('FL','FR','RL','RR'):
                    self.defs['LEG_'+k+'_HIP']=old.FakeNode()
                    self.defs['LEG_'+k+'_KNEE']=old.FakeNode()
        cls.FakeSupervisor=HybridFake
        sys.modules['controller']=types.SimpleNamespace(Supervisor=HybridFake)
        # Ensure base Mission uses the mock controller module.
        sys.modules.pop('drone',None)
        module_spec=importlib.util.spec_from_file_location('_hybrid_controller',
            ROOT/'controllers/hybrid/hybrid.py')
        cls.hybrid=importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(cls.hybrid)

    def test_hybrid_initial_walk_then_autonomous_flight(self):
        h=self.hybrid.HybridMission()
        with tempfile.TemporaryDirectory() as tmp:
            h.captures_dir=Path(tmp)
            self.assertEqual(h.state,'TO_WALK')
            self.assertEqual(len(h.leg_fields),4)
            states=[]
            for _ in range(1300):
                h.robot.step(h.step_ms)
                h.update_rotors()
                h.move_step()
                states.append(h.state)
                if h.state=='FLYING':break
            self.assertIn('WALKING',states)
            self.assertIn('WALK_HOLD',states)
            self.assertIn('TO_FLIGHT',states)
            self.assertEqual(h.state,'FLYING')
            self.assertGreater(h.photo_count,0)
            self.assertGreater(h.walk_phase,0)

    def test_safety_gated_flight_to_walking(self):
        h=self.hybrid.HybridMission()
        h.state='FLYING'
        h.position=[-49,-47,6.0]
        h.evaluate_terrain=lambda force=False: {'status':'UNSAFE','reason':'rough surface'}
        self.assertFalse(h.request_walk())
        self.assertEqual(h.state,'FLYING')
        h.evaluate_terrain=lambda force=False: {'status':'SAFE','reason':'flat'}
        self.assertTrue(h.request_walk())
        self.assertEqual(h.pending_hybrid_route[0].z,6.0)
        self.assertEqual(h.pending_hybrid_route[1].z,h.walk_height)
        self.assertTrue(all(p.mode=='WALK' for p in h.pending_hybrid_route[1:]))
        self.assertEqual(h.state,'TO_WALK')

    def test_walk_force_is_labelled_analytical(self):
        h=self.hybrid.HybridMission()
        h.state='WALKING'
        h.walk_forces['fl'].getValues=lambda: [1.0, 2.0, 3.0, 99.0]
        reading=h.foot_status()
        self.assertEqual(reading['raw_force_xyz_n']['fl'],[1.0,2.0,3.0])
        self.assertEqual(len(reading['raw_force_xyz_n']),4)
        self.assertAlmostEqual(sum(reading['estimated_support_n'].values()),38.259)
        self.assertIn('not a sensor reading',reading['note'])

    def test_configured_sonar_stop_distance(self):
        h=self.hybrid.HybridMission()
        h.begin_walk()
        h.sensor_obstacle_stop_m=2.2
        h.sensor['ultrasonic_front'].getValue=lambda:2.0
        h.robot.time=4.0
        h.sense_obstacles()
        self.assertTrue(h.paused)
        self.assertIn('EMERGENCY STOP',h.warning_message)

    def test_sensor_obstacle_triggers_replanning(self):
        h=self.hybrid.HybridMission()
        h.begin_walk()
        h.sensor['ultrasonic_front'].getValue=lambda :2.4
        h.robot.time=4.0
        h.sense_obstacles()
        self.assertTrue(h.sensor_obstacle is not None or h.warning_message)
        # If path blocked, controller must pause rather than move into the hazard.
        if h.paused:self.assertTrue(h.warning_message)

if __name__=='__main__':unittest.main()
