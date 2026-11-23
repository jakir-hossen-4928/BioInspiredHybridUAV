"""Offline simulator/controller checks; not a replacement for Webots GUI render."""
import csv
import importlib.util
import json
import math
import re
import sys
import tempfile
import types
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
C=ROOT/'controllers'/'drone'
sys.path.insert(0,str(C))
from planner import Planner


class FakeField:
    def __init__(self,value=None):self.value=value if value is not None else [0,0,0]
    def getSFVec3f(self):return self.value
    def setSFVec3f(self,value):self.value=list(value)
    def setSFRotation(self,value):self.value=list(value)
    def importMFNodeFromString(self,i,s):self.value.append(s)

class FakeNode:
    def __init__(self,value=None):
        self.f={'translation':FakeField(value), 'rotation':FakeField([0,0,1,0]),
                'children':FakeField([])}
    def getField(self,name):return self.f[name]
    def remove(self):pass
    def resetPhysics(self):pass

class FakeDevice:
    def getWidth(self):return 1920
    def getHeight(self):return 1080
    def __init__(self,name):self.name=name;self.value=0
    def enable(self,p):self.period=p
    def enablePointCloud(self):pass
    def getValues(self):return [1.0,2.0,3.0]
    def getRollPitchYaw(self):return [0.0,0.0,0.0]
    def getValue(self):return .2
    def getRangeImage(self):return [5.,float('inf'),3.]
    def setPosition(self,value):self.position=value
    def setVelocity(self,value):self.velocity=value
    def set(self,v):self.value=v
    def saveImage(self,file,quality):
        Path(file).write_bytes(b'FAKE_TEST_IMAGE_ONLY')
        return 0

class FakeKeyboard:
    def enable(self,period):pass
    def getKey(self):return -1

class FakeSupervisor:
    def __init__(self):
        s=json.loads((C/'scene.json').read_text())
        self.time=0.
        self.node=FakeNode([*s['start'],.585])
        self.defs={'DYNAMIC_OBSTACLE':FakeNode([0,0,-10]),
                   'GOAL_PAD':FakeNode([*s['goals'][0],.03]),
                   **{key: FakeNode() for key in ('ROTOR_FL','ROTOR_FR','ROTOR_RL','ROTOR_RR')}}
        self.root=FakeNode()
        self.kbd=FakeKeyboard()
        self.devices={}
        self.labels={}
    def getSelf(self):return self.node
    def getRoot(self):return self.root
    def getFromDef(self,n):return self.defs.get(n)
    def getDevice(self,name):
        if name not in self.devices:self.devices[name]=FakeDevice(name)
        return self.devices[name]
    def getBasicTimeStep(self):return 32
    def getKeyboard(self):return self.kbd
    def getTime(self):return self.time
    def step(self,ms):self.time+=ms/1000.;return 0
    def setLabel(self,i,s,*args):self.labels[i]=s


class FarmSurveyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.s=json.loads((C/'scene.json').read_text())
        sys.modules['controller']=types.SimpleNamespace(Supervisor=FakeSupervisor)
        spec=importlib.util.spec_from_file_location('_survey_controller',C/'drone.py')
        cls.mod=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.mod)

    def planner(self):
        return Planner(self.s['obstacles'],resolution=.75,boundary=self.s['bounds_m'],safety=1.2)

    def test_four_missions_route_to_all_stops_and_home(self):
        p=self.planner()
        by={s['id']:s for s in self.s['photo_stops']}
        for mode,ids in self.s['missions'].items():
            with self.subTest(mode=mode):
                start=self.s['start']
                total_length=0
                for stop in ids+['HOME']:
                    dst=self.s['start'] if stop=='HOME' else by[stop]['point']
                    path=p.route(start,dst)
                    self.assertTrue(all(p.clear_segment(a,b)
                                        for a,b in zip(path,path[1:])))
                    total_length+=sum(math.dist(a,b) for a,b in zip(path,path[1:]))
                    start=dst
                self.assertGreater(total_length,170)

    def test_worlds_geometry_local_assets_survey_camera(self):
        worlds=sorted((ROOT/'worlds').glob('*.wbt'))
        self.assertGreaterEqual(len(worlds),6)  # Includes the optional LIVO2 sensor world
        tex_used=set()
        for p in worlds:
            src=p.read_text()
            self.assertTrue(src.startswith('#VRML_SIM R2025a utf8'))
            self.assertIn('DEF DRONE Robot',src)
            self.assertIn('name "survey_camera"',src)
            self.assertIn('rotation 0 1 0 1.57079632679',src)
            self.assertIn('width 3840 height 2160',src)
            self.assertIn('size 1.215 .765 .36',src)
            self.assertIn('translation -49 -47 .585',src)
            self.assertNotIn('Mavic2Pro.proto',src)
            self.assertNotIn('EXTERNPROTO',src)
            self.assertNotRegex(src,r'IndexedFaceSet\s*\{\s*solid\s+')
            self.assertIn('name "Drone"',src)
            self.assertEqual(src.count('PHOTO STOP '),17)
            self.assertNotIn('HingeJoint {',src)
            self.assertNotIn('RotationalMotor {',src)
            for name in ('ROTOR_FL','ROTOR_FR','ROTOR_RL','ROTOR_RR'):
                self.assertEqual(src.count('DEF '+name+' Transform {'),1)
            self.assertIn('boundingObject Box { size 310 310 .32 }',src)
            for m in re.finditer(r'url\s*\[\s*"(\.\./textures/[^"\]]+)"\s*\]',src):
                texture=(p.parent/m.group(1)).resolve()
                self.assertTrue(texture.is_file(),texture)
                tex_used.add(texture.name)
            stripped=re.sub(r'"(?:\\.|[^"\\])*"','""',src)
            stack=[]; matching={'}':'{',']':'['}
            for c in stripped:
                if c in '{[':stack.append(c)
                if c in '}]':
                    self.assertTrue(stack)
                    self.assertEqual(stack.pop(),matching[c])
            self.assertFalse(stack)
        self.assertGreaterEqual(len(tex_used),12)

    def test_auto_hover_capture_and_return_to_home(self):
        m=self.mod.Mission()
        with tempfile.TemporaryDirectory() as temp:
            m.captures_dir=Path(temp)
            self.assertEqual(m.state,'TAKEOFF')
            first_hover_xyz=None
            saw_hover=False
            for step in range(12000):
                # Simulate *Webots* tick timing, not wall time.
                m.robot.step(m.step_ms)
                m.move_step()
                if m.state=='HOVER' and m.stop_index==0:
                    saw_hover=True
                    if first_hover_xyz is None:first_hover_xyz=tuple(m.position)
                    self.assertEqual(tuple(m.position),first_hover_xyz)
                if m.state=='LANDED':break
            self.assertTrue(saw_hover)
            self.assertEqual(m.state,'LANDED')
            self.assertTrue(m.returning)
            self.assertLess(math.dist(m.planar(),self.s['start']),.25)
            self.assertEqual(m.photo_count,2*len(m.route_stops))
            folder=Path(temp)/m.session
            photos=list(folder.glob('*.png'))
            self.assertEqual(len(photos),2*len(m.route_stops))
            with (folder/'photo_manifest.csv').open(newline='') as stream:
                records=list(csv.DictReader(stream))
            self.assertEqual(len(records),2*len(m.route_stops))
            self.assertEqual(records[0]['stop_name'], 'Southwest Wheat A')
            self.assertTrue((folder/'MISSION_GALLERY.html').is_file())
            self.assertEqual(records[0]['camera'],'survey_camera')
            self.assertEqual(records[0]['image_width'],'1920')
            self.assertEqual(records[0]['image_height'],'1080')
            self.assertEqual(records[1]['camera'],'camera')
            self.assertEqual(records[-1]['stop_name'], 'Farm Equipment Yard')
            self.assertEqual(m.sensor['survey_camera'].period,128)


    def test_rotor_visual_animation_and_stop(self):
        m=self.mod.Mission()
        self.assertEqual(len(m.rotor_visuals),4)
        m.robot.step(m.step_ms)
        m.update_rotors()
        angles=[field.value[3] for field, _, _ in m.rotor_visuals]
        self.assertGreater(angles[0],0.1)
        self.assertLess(angles[0],1.0)
        self.assertAlmostEqual(angles[0],angles[3],places=5)
        self.assertAlmostEqual(angles[1],angles[2],places=5)
        self.assertAlmostEqual((angles[0]+angles[1])%(2*math.pi),0,places=5)
        for _ in range(6):
            m.robot.step(m.step_ms)
            m.update_rotors()
        moving=[field.value[3] for field,_,_ in m.rotor_visuals]
        self.assertNotEqual(angles,moving)
        m.state='HOVER'
        m.robot.step(m.step_ms)
        m.update_rotors()
        hovering=[field.value[3] for field,_,_ in m.rotor_visuals]
        self.assertNotEqual(moving,hovering)
        m.state='LANDED'
        m.robot.step(m.step_ms)
        m.update_rotors()
        self.assertEqual(hovering,[field.value[3] for field,_,_ in m.rotor_visuals])
        m.state='FLYING'
        m.paused=True
        m.robot.step(m.step_ms)
        m.update_rotors()
        self.assertEqual(hovering,[field.value[3] for field,_,_ in m.rotor_visuals])

    def test_lossless_manual_png(self):
        m=self.mod.Mission()
        with tempfile.TemporaryDirectory() as temp:
            m.captures_dir=Path(temp)
            self.assertTrue(m.capture('survey_camera',file_format='png'))
            self.assertEqual(len(list((Path(temp)/m.session).glob('*.png'))),1)

    def test_obstacle_replanning_and_route_change(self):
        m=self.mod.Mission()
        start_route=m.waypoints[:]
        m.toggle_obstacle()
        self.assertTrue(m.dynamic_on)
        self.assertNotEqual(start_route,m.waypoints)
        self.assertTrue(all(m.planner.clear_segment(a,b) for a,b in
                            zip(m.waypoints,m.waypoints[1:])))
        m.toggle_obstacle()
        self.assertFalse(m.dynamic_on)
        name=m.mode_name
        m.choose_mission()
        self.assertNotEqual(m.mode_name,name)
        self.assertGreater(len(m.route_stops),4)
        m.skip_stop()
        self.assertEqual(m.stop_index,1)
        m.return_home()
        self.assertTrue(m.returning)
        self.assertEqual(m.goal,self.s['start'])


if __name__=='__main__':
    unittest.main()
