#!/usr/bin/env python3
"""ROS 2 bridge receiving simulated sensor data from Webots over TCP.

Run with `python3 bridge_receiver.py` in an Ubuntu 22.04 + ROS 2 Humble
shell that also sources a workspace containing livox_ros_driver2 messages.

This bridge publishes ROS messages; it is not a SLAM estimator.
"""

import argparse
import math
import os
import socket
import struct
import sys
import threading
import time
import zlib
from pathlib import Path

# Shared source is installed alongside this script for an importable, pure Python protocol.
from .protocol import recv_frame, unpack_points, FrameError

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, Image, CameraInfo, PointCloud2, PointField
    from geometry_msgs.msg import PoseStamped
    from rosgraph_msgs.msg import Clock
    from builtin_interfaces.msg import Time
except ImportError as exc:
    raise SystemExit('ROS 2 Python packages are unavailable. Source /opt/ros/humble/setup.bash: %s' % exc)

try:
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
    HAVE_LIVOX_MSG = True
except ImportError:
    HAVE_LIVOX_MSG = False


PC_RECORD = struct.Struct('<fffffHxx')  # xyz intensity time(s) ring


def stamp_from_seconds(value):
    sec = int(math.floor(float(value)))
    nsec = int(round((float(value) - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    msg = Time()
    msg.sec, msg.nanosec = sec, nsec
    return msg


def quaternion(roll, pitch, yaw):
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


class SensorBridge(Node):
    def __init__(self, add_sim_gravity=False):
        super().__init__('webots_fast_livo_bridge')
        self.add_sim_gravity = add_sim_gravity
        # FAST-LIVO2's depth-only subscriptions request reliable delivery.
        self.imu_pub = self.create_publisher(Imu, '/livox/imu', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/webots/lidar/points', qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, '/webots/slam/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/webots/slam/camera_info', qos_profile_sensor_data)
        self.truth_pub = self.create_publisher(PoseStamped, '/webots/ground_truth', qos_profile_sensor_data)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.livox_pub = (self.create_publisher(CustomMsg, '/livox/lidar', 10)
                          if HAVE_LIVOX_MSG else None)
        self.count = {'lidar':0,'imu':0,'image':0,'gps':0}
        self.latest_sim_time = -1
        if HAVE_LIVOX_MSG:
            self.get_logger().info('Livox CustomMsg available: /livox/lidar will publish')
        else:
            self.get_logger().warning('livox_ros_driver2 missing: /livox/lidar NOT published; only PointCloud2')

    @staticmethod
    def _header(stamp, frame):
        # Header class from the regular ROS message package.
        from std_msgs.msg import Header
        header = Header()
        header.stamp = stamp
        header.frame_id = frame
        return header

    def ingest(self, meta, payload):
        kind = meta.get('type')
        value = float(meta['sim_time_s'])
        stamp = stamp_from_seconds(value)
        self.clock_pub.publish(Clock(clock=stamp))
        if value < self.latest_sim_time - 0.02:
            self.get_logger().warning('Webots time reset: restart FAST-LIVO2 after resetting the world')
        self.latest_sim_time = value
        if kind == 'imu': self._imu(meta, stamp)
        elif kind == 'lidar': self._lidar(meta, payload, stamp)
        elif kind == 'image': self._image(meta, payload, stamp)
        elif kind == 'gps': self._gps(meta, stamp)
        else: self.get_logger().warning('Unknown stream type %r' % kind)
        if kind in self.count: self.count[kind] += 1
        if kind == 'lidar' and self.count['lidar'] % 20 == 0:
            self.get_logger().info('frames IMU=%d LiDAR=%d camera=%d GPS=%d' % tuple(
                self.count[k] for k in ('imu','lidar','image','gps')))

    def _imu(self, m, stamp):
        msg = Imu()
        msg.header = self._header(stamp, 'imu_link')
        roll, pitch, yaw = map(float, m['rpy'])
        q = quaternion(roll, pitch, yaw)
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = m['omega_rad_s']
        ax, ay, az = m['accel_m_s2']
        if self.add_sim_gravity:
            # Add world-up specific force in IMU coordinates for the zero-gravity worlds.
            ax -= 9.81 * math.sin(pitch)
            ay += 9.81 * math.cos(pitch) * math.sin(roll)
            az += 9.81 * math.cos(pitch) * math.cos(roll)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = ax, ay, az
        # Covariances are not calibrated. A ROS all-zero covariance means unknown.
        self.imu_pub.publish(msg)

    def _lidar(self, m, payload, stamp):
        points = unpack_points(payload)
        cloud = PointCloud2()
        cloud.header = self._header(stamp, 'livox_frame')
        cloud.height, cloud.width = 1, len(points)
        cloud.is_bigendian = False
        cloud.is_dense = True
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='time', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1),
        ]
        cloud.point_step = PC_RECORD.size
        cloud.row_step = cloud.width * cloud.point_step
        cloud.data = b''.join(PC_RECORD.pack(x, y, z, float(refl), float(offset)/1e9, int(layer))
                              for x,y,z,offset,refl,tag,layer in points)
        self.cloud_pub.publish(cloud)
        if self.livox_pub:
            msg = CustomMsg()
            msg.header = self._header(stamp, 'livox_frame')
            msg.timebase = int(float(m['sim_time_s']) * 1e9)
            msg.point_num = len(points)
            msg.lidar_id = 1
            msg.rsvd = [0,0,0]
            msg.points = [CustomPoint(offset_time=offset, x=x, y=y, z=z,
                                      reflectivity=refl, tag=tag, line=layer)
                          for x,y,z,offset,refl,tag,layer in points]
            self.livox_pub.publish(msg)

    def _image(self, m, payload, stamp):
        if m.get('compression') != 'zlib' or m.get('encoding') != 'bgra8':
            raise FrameError('unsupported image format')
        width, height = int(m['width']), int(m['height'])
        if width <= 0 or height <= 0 or width * height > 4096*4096:
            raise FrameError('invalid camera dimensions')
        raw = zlib.decompress(payload)
        if len(raw) != width * height * 4:
            raise FrameError('invalid camera data length')
        try:
            import numpy as np
            bgr = np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 4)[:,:,:3].tobytes()
        except ImportError:
            # Preserve the blue-green-red order in Webots BGRA buffers.
            bgr = bytearray(width*height*3)
            view = memoryview(raw)
            for i in range(width*height):
                bgr[3*i:3*i+3] = view[4*i:4*i+3]
            bgr = bytes(bgr)
        msg = Image()
        msg.header = self._header(stamp, 'slam_camera_optical_frame')
        msg.height, msg.width = height, width
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = width * 3
        msg.data = bgr
        self.image_pub.publish(msg)
        fov = float(m['horizontal_fov_rad'])
        fx = float(width) / (2 * math.tan(fov / 2))
        fy = fx
        cx, cy = width/2 - .5, height/2 - .5
        info = CameraInfo()
        info.header = msg.header
        info.width, info.height = width, height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [fx,0.0,cx, 0.0,fy,cy, 0.0,0.0,1.0]
        info.r = [1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0]
        info.p = [fx,0.0,cx,0.0, 0.0,fy,cy,0.0, 0.0,0.0,1.0,0.0]
        self.info_pub.publish(info)

    def _gps(self, m, stamp):
        msg = PoseStamped()
        msg.header = self._header(stamp, 'map')
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = m['position_m']
        q = quaternion(0, 0, float(m.get('yaw_rad', 0)))
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        self.truth_pub.publish(msg)


def listen(node, host, port, stop):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)
    server.settimeout(.5)
    node.get_logger().info('Listening for Webots on %s:%d' % (host, port))
    while not stop.is_set():
        try:
            conn, addr = server.accept()
        except socket.timeout:
            continue
        except OSError:
            break
        node.get_logger().info('Webots connected from %s' % (addr,))
        with conn:
            conn.settimeout(4)
            while not stop.is_set():
                try:
                    meta, payload = recv_frame(conn)
                    node.ingest(meta, payload)
                except (EOFError, OSError):
                    break
                except (FrameError, ValueError, KeyError, TypeError, zlib.error) as exc:
                    node.get_logger().error('Dropping invalid message: %s' % exc)
                    break
        node.get_logger().info('Waiting for reconnection')
    server.close()


def main():
    parser = argparse.ArgumentParser(description='Bridge Webots sensor messages to ROS2 topics')
    parser.add_argument('--host', default='127.0.0.1', help='Host to bind; use 0.0.0.0 only on trusted networks')
    parser.add_argument('--port', type=int, default=15882)
    parser.add_argument('--add-sim-gravity', action='store_true')
    from rclpy.utilities import remove_ros_args
    args = parser.parse_args(remove_ros_args()[1:])
    rclpy.init()
    node = SensorBridge(add_sim_gravity=args.add_sim_gravity)
    stop = threading.Event()
    thread = threading.Thread(target=listen, args=(node,args.host,args.port,stop), daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
