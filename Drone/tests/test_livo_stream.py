"""Tests for the real byte-stream bridge, independent of ROS 2 and Webots GUI."""
import json
import math
import os
import socket
import struct
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE/'controllers/drone'))
from livo_protocol import (POINT, HEADER, FrameError, pack_point_cloud,
                           unpack_points, encode_frame, decode_frame_bytes, recv_frame)
from fast_livo_stream import FastLivoStreamer


class FakeSensor:
    def __init__(self, values=(0,0,0)):
        self.values=values
    def getValues(self): return self.values
    def getRollPitchYaw(self): return (0.01, -0.02, 0.03)

class FakeLidar:
    def enablePointCloud(self): self.enabled=True
    def getHorizontalResolution(self): return 3
    def getPointCloud(self):
        return [SimpleNamespace(x=float(i+1), y=1.0, z=.4) for i in range(6)]

class FakeCamera:
    def getWidth(self): return 4
    def getHeight(self): return 3
    def getFov(self): return 1.05
    def getImage(self): return bytes([40, 80, 120, 255]) * 12


class ProtocolTests(unittest.TestCase):
    def test_metadata_roundtrip(self):
        encoded=encode_frame({'type':'imu', 'sim_time_s':.32}, b'hello')
        meta, body=decode_frame_bytes(encoded)
        self.assertEqual(meta['type'],'imu')
        self.assertEqual(body,b'hello')

    def test_reject_wrong_magic(self):
        encoded=encode_frame({'type':'test'})
        with self.assertRaises(FrameError): decode_frame_bytes(b'NOPE'+encoded[4:])

    def test_reject_oversize(self):
        with self.assertRaises(FrameError): decode_frame_bytes(HEADER.pack(b'WBL2',1, 99_999_999,0))

    def test_cloud_filters_nonfinite_and_zero_range(self):
        pts=[SimpleNamespace(x=1.0, y=0,z=0),
             SimpleNamespace(x=float('inf'),y=0,z=0),
             SimpleNamespace(x=0,y=0,z=0),
             SimpleNamespace(x=3.0,y=1,z=2)]
        data,count=pack_point_cloud(pts,2)
        self.assertEqual(count,2)
        self.assertEqual(len(data),2*POINT.size)
        self.assertEqual(POINT.size,19)
        self.assertEqual(unpack_points(data)[0][0],1.0)

    def test_simulated_ray_timing_is_opt_in(self):
        pts=[SimpleNamespace(x=1.0,y=0.0,z=0.0) for _ in range(4)]
        raw,_=pack_point_cloud(pts,4)
        self.assertEqual([r[3] for r in unpack_points(raw)],[0,0,0,0])
        assumed,_=pack_point_cloud(pts,4,40_000_000)
        self.assertEqual([r[3] for r in unpack_points(assumed)], [0,10_000_000,20_000_000,30_000_000])

    def test_stream_socket_frame_roundtrip(self):
        a,b=socket.socketpair()
        try:
            raw=encode_frame({'type':'gps','sim_time_s':1.25},b'xyz')
            for i in range(0,len(raw),3):a.sendall(raw[i:i+3])
            m,data=recv_frame(b)
            self.assertEqual(m['type'],'gps')
            self.assertEqual(data,b'xyz')
        finally:a.close();b.close()

    def test_writer_and_mock_webots_devices(self):
        host='127.0.0.1'
        server=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        server.bind((host,0))
        port=server.getsockname()[1]
        server.listen(1)
        server.settimeout(4)
        result=[]
        done=threading.Event()
        def receive():
            try:
                conn,_=server.accept()
                with conn:
                    conn.settimeout(4)
                    while len(result)<4:
                        meta,raw=recv_frame(conn)
                        result.append((meta,raw))
                        if {m['type'] for m,_ in result}=={'imu','lidar','image','gps'}:break
            finally:done.set();server.close()
        thread=threading.Thread(target=receive,daemon=True)
        thread.start()
        sensors={'lidar':FakeLidar(), 'slam_camera':FakeCamera(),
                 'inertial unit':FakeSensor(), 'gyro':FakeSensor((.02,.01,-.01)),
                 'accelerometer':FakeSensor((0,0,9.8)), 'gps':FakeSensor((1,2,3))}
        options={'fast_livo_enabled':True,'fast_livo_host':host,'fast_livo_port':port,
                 'fast_livo_imu_interval_ms':16,'fast_livo_lidar_interval_ms':16,
                 'fast_livo_camera_interval_ms':16}
        obj=FastLivoStreamer(object(), sensors, options, 16)
        try:
            for i in range(250):
                if obj.writer.connected: break
                time.sleep(.01)
            self.assertTrue(obj.writer.connected)
            obj.update(.32, [1,2,3], .1)
            self.assertTrue(done.wait(4))
            kinds={m['type'] for m,_ in result}
            self.assertEqual(kinds,{'imu','lidar','image','gps'})
            image_meta,image_data=next((m,d) for m,d in result if m['type']=='image')
            import zlib
            self.assertEqual(zlib.decompress(image_data), FakeCamera().getImage())
            cloud_meta,cloud_data=next((m,d) for m,d in result if m['type']=='lidar')
            self.assertEqual(cloud_meta['point_count'],6)
            self.assertEqual(len(unpack_points(cloud_data)),6)
        finally:
            obj.close()
            thread.join(timeout=2)

    def test_monitor_outputs_actual_ply_and_bitmap(self):
        import tempfile
        sys.path.insert(0, str(HERE/'integration'))
        from monitor import save_ply, save_bmp
        with tempfile.TemporaryDirectory() as td:
            ply=Path(td)/'scan.ply'
            records=[(1.0,2.0,3.0,0,180,0,0),(2.0,3.0,4.0,0,180,0,1)]
            save_ply(ply,records)
            self.assertIn('element vertex 2',ply.read_text())
            bmp=Path(td)/'frame.bmp'
            save_bmp(bmp, bytes([30,60,90,255])*12,4,3)
            self.assertEqual(bmp.read_bytes()[:2],b'BM')
            self.assertGreater(bmp.stat().st_size,54)

    def test_world_contains_dedicated_slam_camera_preserves_4k(self):
        txt=(HERE/'worlds/livo2.wbt').read_text()
        self.assertIn('name "slam_camera"',txt)
        self.assertIn('name "survey_camera"',txt)
        self.assertIn('width 3840 height 2160',txt)
        self.assertIn('basicTimeStep 16',txt)
        self.assertIn('numberOfLayers 24',txt)

    def test_controller_calls_bridge_after_move(self):
        txt=(HERE/'controllers/drone/drone.py').read_text()
        self.assertIn('self.livo_stream.update(t, self.position, self.yaw)',txt)
        self.assertIn('FastLivoStreamer(self.robot, self.sensor',txt)

if __name__=='__main__':unittest.main()
