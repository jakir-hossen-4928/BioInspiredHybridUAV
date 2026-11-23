"""ROS 2 integration package checks runnable without a ROS 2 installation."""
import sys
import unittest
import tempfile
import importlib.util
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT/'ros2_ws/src/drone_ros_bridge'


class ROS2StructureTests(unittest.TestCase):
    def test_ament_package_files(self):
        for relative in ('package.xml', 'setup.py', 'setup.cfg',
                         'drone_ros_bridge/receiver.py', 'drone_ros_bridge/protocol.py',
                         'launch/bridge.launch.py', 'launch/fast_livo2.launch.py'):
            self.assertTrue((SRC/relative).is_file(), relative)

    def test_topics_are_not_fake_estimator_results(self):
        text=(SRC/'drone_ros_bridge/receiver.py').read_text()
        for path in ('/livox/imu','/livox/lidar','/webots/lidar/points',
                     '/webots/slam/image_raw','/webots/slam/camera_info','/webots/ground_truth'):
            self.assertIn(path,text)
        self.assertNotIn('create_publisher(Odometry',text)

    def test_hybrid_world_uses_8ms_sampling_and_sensor_names(self):
        w=(ROOT/'worlds/hybrid_ros2.wbt').read_text()
        for name in ('basicTimeStep 8','controller "hybrid"','name "slam_camera"',
                     'name "lidar"', 'name "gyro"', 'name "accelerometer"'):
            self.assertIn(name,w)

    def test_protocol_roundtrip(self):
        sys.path.insert(0,str(SRC))
        from drone_ros_bridge.protocol import encode_frame,decode_frame_bytes,pack_point_cloud,unpack_points
        b=encode_frame({'type':'imu','sim_time_s':1.25})
        meta, data=decode_frame_bytes(b)
        self.assertEqual(meta['type'],'imu')
        self.assertEqual(meta['sim_time_s'],1.25)
        self.assertEqual(data,b'')
        class Point:
            x=1.2;y=2.5;z=3.4
        packed,num=pack_point_cloud([Point()],360)
        self.assertEqual(num,1)
        self.assertEqual(len(unpack_points(packed)),1)

    def test_prepare_config_replaces_external_calibration(self):
        try: import yaml
        except ImportError: self.skipTest('PyYAML missing')
        module_path=ROOT/'integration/ros2/prepare_fast_config.py'
        spec=importlib.util.spec_from_file_location('prepare_fast_config',module_path)
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        mid={'/**':{'ros__parameters':{
            'common':{'lid_topic':'/wrong','img_topic':'/wrong','imu_topic':'/wrong'},
            'extrin_calib':{'extrinsic_T':[8,8,8], 'Pcl':[9,9,9]},
            'preprocess':{'lidar_type':8},
            'time_offset':{'imu_time_offset':1.0,'img_time_offset':999.0},
            'rolling_shutter':{'enable':True,'td_estimate_enable':True},
        }}}
        cam={'/**':{'ros__parameters':{'cam_width':1280}}}
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            (p/'mid360_lvi.yaml').write_text(yaml.safe_dump(mid))
            (p/'camera_pinhole.yaml').write_text(yaml.safe_dump(cam))
            generated=module.update_configs(p/'mid360_lvi.yaml',p/'camera_pinhole.yaml',p/'out')
            new=yaml.safe_load(generated[0].read_text())['/**']['ros__parameters']
            cp=yaml.safe_load(generated[1].read_text())['/**']['ros__parameters']
            self.assertEqual(new['common']['lid_topic'],'/livox/lidar')
            self.assertEqual(new['common']['imu_topic'],'/livox/imu')
            self.assertEqual(new['extrin_calib']['extrinsic_T'],[0.0]*3)
            self.assertFalse(new['rolling_shutter']['enable'])
            self.assertEqual(cp['cam_width'],640)
            self.assertEqual(cp['cam_height'],480)
            self.assertTrue(new['use_sim_time'])

    def test_setup_scripts_present(self):
        for sh in ('setup_ros2_workspace.sh','run_ros2_bridge.sh','run_ros2_fast_livo2.sh',
                   'check_ros2_topics.sh','fetch_fast_livo2_sources.sh'):
            self.assertTrue((ROOT/'integration/ros2'/sh).exists())

if __name__=='__main__':unittest.main()
