"""Pure depth-based terrain inference tests; does not claim Webots GPU visual QA."""
import importlib.util
import math
import sys
import tempfile
import types
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
C=ROOT/'controllers'/'drone'
sys.path.insert(0,str(C))
from terrain_perception import analyse_depth_image, pressure_hpa_from_altitude, HeightGrid
from test_navigation import FakeSupervisor

W,H,FOV=64,48,.70
SENSOR_HEIGHT=13.0-.38
TANH=math.tan(FOV/2)
TANV=TANH*H/W

def synthetic_depth(slope=0, amplitude=0, height=0, empty=False):
    result=[]
    for j in range(H):
        for i in range(W):
            nx=(i+.5-W/2)/(W/2)
            ny=(j+.5-H/2)/(H/2)
            # solve d=Z_sensor - z, z=height+slope*x+roughness(x,y)
            r=SENSOR_HEIGHT-height
            for _ in range(3):
                x=-ny*TANV*r
                rough=amplitude*(math.sin(5*x)+math.cos(2*nx*TANH*r))
                r=SENSOR_HEIGHT-(height+slope*x+rough)
            result.append(float('inf') if empty else r)
    return result

def classify(values):
    return analyse_depth_image(values,W,H,FOV,[8.,-4.,13.],0.0)

class TerrainTests(unittest.TestCase):
    def test_level_ground_safe_from_valid_depth(self):
        report=classify(synthetic_depth())
        self.assertEqual(report['status'],'SAFE', report)
        self.assertGreater(report['sample_count'],60)
        self.assertLess(report['roughness_m'],.0001)
        self.assertLess(report['slope_deg'],.1)
        self.assertGreater(len(report['points']),60)

    def test_steep_ground_rejected(self):
        report=classify(synthetic_depth(slope=.35))
        self.assertEqual(report['status'],'UNSAFE')
        self.assertGreater(report['slope_deg'],10)
        self.assertIn('slope',report['reason'])

    def test_rough_ground_rejected(self):
        report=classify(synthetic_depth(amplitude=.35))
        self.assertEqual(report['status'],'UNSAFE')
        self.assertGreater(report['roughness_m'],.16)

    def test_elevated_flat_roof_rejected(self):
        report=classify(synthetic_depth(height=4.2))
        self.assertEqual(report['status'],'UNSAFE')
        self.assertIn('expected ground level',report['reason'])

    def test_no_range_values_is_not_safe(self):
        report=classify(synthetic_depth(empty=True))
        self.assertEqual(report['status'],'NO_DATA')
        self.assertEqual(report['sample_count'],0)

    def test_isa_and_observed_map(self):
        self.assertAlmostEqual(pressure_hpa_from_altitude(0),1013.25,places=1)
        self.assertLess(pressure_hpa_from_altitude(1000),pressure_hpa_from_altitude(0))
        grid=HeightGrid()
        grid.extend([(1.2,3.4,.1),(1.3,3.4,.3),(-1.,5.,2.)])
        self.assertEqual(len(grid.cells),2)
        self.assertIn('svg',grid.svg())
        self.assertEqual(len(list(grid.csv_rows())),2)

    @classmethod
    def setUpClass(cls):
        sys.modules['controller']=types.SimpleNamespace(Supervisor=FakeSupervisor)
        spec=importlib.util.spec_from_file_location('_terrain_controller',C/'drone.py')
        cls.mod=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.mod)

    def fake_mission_with_depth(self, values):
        m=self.mod.Mission()
        td=tempfile.TemporaryDirectory()
        self.addCleanup(td.cleanup)
        m.captures_dir=Path(td.name)
        m.sensor_dashboard=Path(td.name)/'dashboard.html'
        class FakeDepth:
            def getWidth(self):return W
            def getHeight(self):return H
            def getFov(self):return FOV
            def getRangeImage(self):return values
        m.sensor['terrain_depth']=FakeDepth()
        m.position=[8.0,-4.0,13.0]
        m.state='HOVER'
        m.yaw=0.0
        return m

    def test_landing_approved_only_for_safe_measured_depth(self):
        m=self.fake_mission_with_depth(synthetic_depth())
        self.assertTrue(m.request_sensor_checked_landing())
        self.assertEqual(m.state,'LANDING')
        self.assertIn('APPROVED',m.landing_decision)

    def test_landing_blocked_by_rough_depth(self):
        m=self.fake_mission_with_depth(synthetic_depth(amplitude=.35))
        self.assertFalse(m.request_sensor_checked_landing())
        self.assertEqual(m.state,'HOVER')
        self.assertIn('REJECTED',m.landing_decision)

    def test_landing_blocked_without_sensor_data(self):
        m=self.fake_mission_with_depth(synthetic_depth(empty=True))
        self.assertFalse(m.request_sensor_checked_landing())
        self.assertEqual(m.state,'HOVER')

    def test_live_dashboard_has_sensor_labels_and_log(self):
        m=self.fake_mission_with_depth(synthetic_depth())
        with tempfile.TemporaryDirectory() as td:
            m.captures_dir=Path(td)
            m.sensor_dashboard=Path(td)/'dashboard.html'
            m.evaluate_terrain(force=True)
            m.write_sensor_dashboard(force=True)
            self.assertTrue(m.sensor_dashboard.is_file())
            doc=m.sensor_dashboard.read_text()
            self.assertIn('LANDING FOOTPRINT SAFE',doc)
            self.assertIn('Barometric estimate',doc)
            self.assertIn('NOT force in Newtons',doc)
            folder=Path(td)/m.session
            self.assertTrue((folder/'SENSOR_TELEMETRY.csv').is_file())
            self.assertTrue((folder/'OBSERVED_HEIGHT_MAP.csv').is_file())

    def test_all_worlds_include_downward_metric_rangefinder(self):
        worlds=list((ROOT/'worlds').glob('*.wbt'))
        self.assertGreaterEqual(len(worlds),6)  # Includes the optional LIVO2 sensor world
        for world in worlds:
            src=world.read_text()
            self.assertIn('name "terrain_depth"',src)
            self.assertIn('RangeFinder {',src)
            self.assertIn('name "Drone"',src)

if __name__=='__main__':unittest.main()
