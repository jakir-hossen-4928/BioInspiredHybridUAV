"""Webots-to-ROS 2 sensor stream for an external FAST-LIVO2 process.

This code is a transport adapter, *not* an implementation of FAST-LIVO2.
It sends Webots LiDAR geometry, IMU measurements, and a low-bandwidth RGB
camera over TCP.  It does not estimate poses or reconstruct a SLAM map.
"""

import math
import queue
import socket
import threading
import time
import zlib

from livo_protocol import encode_frame, pack_point_cloud


class _TCPWriter:
    def __init__(self, host, port):
        self.host, self.port = str(host), int(port)
        self.queue = queue.Queue(maxsize=36)
        self.connected = False
        self.sent = 0
        self.dropped = 0
        self._stopped = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name='webots_livo_tcp')
        self._thread.start()

    def submit(self, metadata, data=b''):
        if not self.connected:
            return False
        try:
            frame = encode_frame(metadata, data)
            self.queue.put_nowait(frame)
            return True
        except (queue.Full, ValueError, TypeError):
            self.dropped += 1
            return False

    def _loop(self):
        while not self._stopped.is_set():
            try:
                with socket.create_connection((self.host, self.port), timeout=.8) as sock:
                    sock.settimeout(3)
                    self.connected = True
                    print('[LIVO STREAM] Connected to ROS receiver %s:%d' % (self.host, self.port))
                    while not self._stopped.is_set():
                        try:
                            frame = self.queue.get(timeout=.3)
                        except queue.Empty:
                            continue
                        sock.sendall(frame)
                        self.sent += 1
            except (OSError, TimeoutError) as exc:
                if self.connected:
                    print('[LIVO STREAM] Connection lost:', exc)
            finally:
                self.connected = False
                # Do not send old sensor frames after a reconnection.
                while True:
                    try: self.queue.get_nowait()
                    except queue.Empty: break
            self._stopped.wait(1.)

    def close(self):
        self._stopped.set()


class FastLivoStreamer:
    def __init__(self, robot, sensors, settings, step_ms):
        self.robot = robot
        self.sensors = sensors
        self.settings = settings
        self.step_ms = step_ms
        self.enabled = bool(settings.get('fast_livo_enabled', True))
        self.last_imu = -1e9
        self.last_lidar = -1e9
        self.last_camera = -1e9
        self.last_pose = -1e9
        self.imu_period = max(step_ms, int(settings.get('fast_livo_imu_interval_ms', step_ms))) / 1000
        self.lidar_period = max(step_ms, int(settings.get('fast_livo_lidar_interval_ms', 160))) / 1000
        self.camera_period = max(step_ms, int(settings.get('fast_livo_camera_interval_ms', 320))) / 1000
        self.time_assumption_ns = int(max(0, float(settings.get('fast_livo_assumed_scan_period_ms', 0))) * 1e6)
        self.writer = None
        if not self.enabled:
            return
        lidar = sensors.get('lidar')
        if lidar is not None:
            try:
                lidar.enablePointCloud()
            except Exception as exc:
                print('[LIVO STREAM] Could not enable point cloud:', exc)
        if sensors.get('slam_camera') is None:
            print('[LIVO STREAM] slam_camera missing; use worlds/livo2.wbt for LiDAR+IMU+RGB')
        self.writer = _TCPWriter(settings.get('fast_livo_host', '127.0.0.1'),
                                 settings.get('fast_livo_port', 15882))
        print('[LIVO STREAM] External FAST-LIVO2 receiver optional; simulated sensor feed only')

    @staticmethod
    def _valid(values, count):
        return list(values) if values is not None and len(values) == count and all(
            math.isfinite(float(x)) for x in values) else None

    def update(self, t, true_position=None, yaw=None):
        if not self.writer or not self.writer.connected:
            return
        if t - self.last_imu >= self.imu_period - 1e-6:
            self.last_imu = t
            imu = self.sensors.get('inertial unit')
            gyro = self.sensors.get('gyro')
            accel = self.sensors.get('accelerometer')
            if imu and gyro and accel:
                try:
                    rpy = self._valid(imu.getRollPitchYaw(), 3)
                    omega = self._valid(gyro.getValues(), 3)
                    linear = self._valid(accel.getValues(), 3)
                    if rpy and omega and linear:
                        self.writer.submit({'type': 'imu', 'sim_time_s': t,
                                            'rpy': rpy, 'omega_rad_s': omega,
                                            'accel_m_s2': linear,
                                            'frame': 'imu_link'})
                except Exception as exc:
                    print('[LIVO STREAM] IMU read error:', exc)
        if t - self.last_pose >= .25:
            self.last_pose = t
            gps = self.sensors.get('gps')
            try:
                pos = self._valid(gps.getValues(), 3) if gps else None
                if pos:
                    self.writer.submit({'type': 'gps', 'sim_time_s': t,
                                        'position_m': pos, 'yaw_rad': float(yaw or 0),
                                        'frame': 'map'})
            except Exception: pass
        if t - self.last_lidar >= self.lidar_period - 1e-6:
            self.last_lidar = t
            lidar = self.sensors.get('lidar')
            if lidar:
                try:
                    points = lidar.getPointCloud()
                    packed, count = pack_point_cloud(points, lidar.getHorizontalResolution(),
                                                      self.time_assumption_ns)
                    if count:
                        self.writer.submit({'type': 'lidar', 'sim_time_s': t,
                                            'frame': 'livox_frame', 'point_count': count,
                                            'point_record_bytes': 19,
                                            'temporal_offsets': 'assumed' if self.time_assumption_ns else 'none',
                                            'reflectivity': 'fixed_simulated'}, packed)
                except Exception as exc:
                    print('[LIVO STREAM] LiDAR read error:', exc)
        if t - self.last_camera >= self.camera_period - 1e-6:
            self.last_camera = t
            cam = self.sensors.get('slam_camera')
            if cam:
                try:
                    raw = cam.getImage()
                    width, height = cam.getWidth(), cam.getHeight()
                    if raw is not None and len(raw) == width * height * 4:
                        self.writer.submit({'type': 'image', 'sim_time_s': t,
                                            'frame': 'slam_camera_optical_frame',
                                            'width': width, 'height': height,
                                            'horizontal_fov_rad': float(cam.getFov()),
                                            'encoding': 'bgra8', 'compression': 'zlib'},
                                           zlib.compress(bytes(raw), level=1))
                except Exception as exc:
                    print('[LIVO STREAM] Camera read error:', exc)

    def close(self):
        if self.writer: self.writer.close()
