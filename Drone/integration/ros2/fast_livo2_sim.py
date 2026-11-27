#!/usr/bin/env python3
"""Simulated LiDAR-inertial estimator for the Webots demo.

This is a stand-in for FAST-LIVO2, not FAST-LIVO2: it consumes the same TCP
sensor stream the ROS 2 bridge would publish (`/livox/imu`, `/livox/lidar`,
`/webots/ground_truth`), predicts with the IMU and corrects with scan-to-map
ICP, then writes the pose track (`/aft_mapped_to_init` equivalent) and the
registered map (`/cloud_registered` equivalent). Camera frames are ignored,
so this is LiDAR-inertial only - no visual term.

Two limits measured on this world, quoted in the documentation:
  * the kinematic Supervisor reports zero accelerometer/gyro values, so the
    IMU branch only exercises the interface;
  * at survey altitude the scans are ground-dominated, which does not
    constrain horizontal motion, so the tracked pose drifts. Unit tests cover
    the registration maths; convergence on a real flight still needs the
    external estimator.

Pure standard library so it runs on Windows next to Webots:

    python fast_livo2_sim.py --port 15882 --seconds 300 --out runs/livo
"""
import argparse
import json
import math
import socket
import sys
import time
from pathlib import Path

from livo_protocol import recv_frame, unpack_points

VOXEL = 1.0


def key(point, size):
    return (int(round(point[0] / size)), int(round(point[1] / size)),
            int(round(point[2] / size)))


def downsample(points, size=VOXEL):
    seen, out = set(), []
    for x, y, z, *_ in points:
        cell = key((x, y, z), size)
        if cell in seen:
            continue
        seen.add(cell)
        out.append((x, y, z))
    return out


def solve4(matrix, vector):
    """Gaussian elimination for the 4x4 normal equations of the ICP step."""
    size = 4
    rows = [list(matrix[i]) + [vector[i]] for i in range(size)]
    for col in range(size):
        pivot = max(range(col, size), key=lambda r: abs(rows[r][col]))
        if abs(rows[pivot][col]) < 1e-12:
            return None
        rows[col], rows[pivot] = rows[pivot], rows[col]
        scale = rows[col][col]
        rows[col] = [value / scale for value in rows[col]]
        for other in range(size):
            if other == col:
                continue
            factor = rows[other][col]
            rows[other] = [a - factor * b for a, b in zip(rows[other], rows[col])]
    return [rows[i][size] for i in range(size)]


def transform(points, tx, ty, tz, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(c * x - s * y + tx, s * x + c * y + ty, z + tz) for x, y, z in points]


class ScanMap:
    """Voxel-hashed world points; nearest neighbour within one voxel ring."""

    def __init__(self, size=VOXEL):
        self.size = size
        self.cells = {}

    def add(self, points, tx, ty, tz, yaw):
        for point in transform(points, tx, ty, tz, yaw):
            cell = key(point, self.size)
            bucket = self.cells.setdefault(cell, [])
            if len(bucket) < 4:
                bucket.append(point)

    def nearest(self, point, radius=1.6):
        base = key(point, self.size)
        best, best_d2 = None, radius * radius
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for candidate in self.cells.get((base[0] + dx, base[1] + dy,
                                                     base[2] + dz), ()):
                        d2 = ((candidate[0] - point[0]) ** 2
                              + (candidate[1] - point[1]) ** 2
                              + (candidate[2] - point[2]) ** 2)
                        if d2 < best_d2:
                            best, best_d2 = candidate, d2
        return best

    def all_points(self):
        return [p for bucket in self.cells.values() for p in bucket]


def icp_step(scan, scan_map, tx, ty, tz, yaw, iterations=6):
    """Point-to-point ICP of one scan against the accumulated map.

    Minimises ||R(yaw)p + t - q|| over (tx, ty, tz, yaw) with Gauss-Newton.
    """
    for _ in range(iterations):
        weights = [[0.0] * 4 for _ in range(4)]
        rhs = [0.0] * 4
        used = 0
        c, s = math.cos(yaw), math.sin(yaw)
        for x, y, z in scan:
            wx, wy, wz = c * x - s * y + tx, s * x + c * y + ty, z + tz
            match = scan_map.nearest((wx, wy, wz))
            if match is None:
                continue
            residual = (wx - match[0], wy - match[1], wz - match[2])
            rows = ((1.0, 0.0, 0.0, -s * x - c * y),
                    (0.0, 1.0, 0.0, c * x - s * y),
                    (0.0, 0.0, 1.0, 0.0))
            for row in range(4):
                for col in range(4):
                    weights[row][col] += sum(rows[k][row] * rows[k][col] for k in range(3))
                rhs[row] -= sum(rows[k][row] * residual[k] for k in range(3))
            used += 1
        if used < 30:
            return tx, ty, tz, yaw, used
        delta = solve4(weights, rhs)
        if delta is None:
            return tx, ty, tz, yaw, used
        tx += delta[0]
        ty += delta[1]
        tz += delta[2]
        yaw += delta[3]
        if abs(delta[0]) + abs(delta[1]) + abs(delta[2]) < 1e-3 and abs(delta[3]) < 1e-4:
            break
    return tx, ty, tz, yaw, used


class Estimator:
    def __init__(self, out):
        self.out = out
        self.pose = [0.0, 0.0, 0.0, 0.0]      # x, y, z, yaw
        self.velocity = [0.0, 0.0, 0.0]
        self.last_imu_time = None
        self.map = ScanMap()
        self.track = []                        # (t, est xyz, truth xyz)
        self.lidar_count = 0
        self.corrected = 0
        self.prior_delta = (0.0, 0.0, 0.0, 0.0)
        self.initialized = False

    def imu(self, meta):
        accel = meta['accel_m_s2']
        omega = meta['omega_rad_s']
        stamp = float(meta['sim_time_s'])
        if self.last_imu_time is None:
            self.last_imu_time = stamp
            return
        dt = stamp - self.last_imu_time
        self.last_imu_time = stamp
        if not 0.0 < dt < 0.2:
            return
        yaw = self.pose[3]
        c, s = math.cos(yaw), math.sin(yaw)
        # World gravity is zero in this world, so the accelerometer is the
        # kinematic acceleration of the body; rotate it into world axes.
        ax, ay, az = accel
        self.velocity[0] += (c * ax - s * ay) * dt
        self.velocity[1] += (s * ax + c * ay) * dt
        self.velocity[2] += az * dt
        self.pose[0] += self.velocity[0] * dt
        self.pose[1] += self.velocity[1] * dt
        self.pose[2] += self.velocity[2] * dt
        self.pose[3] += omega[2] * dt

    def lidar(self, meta, payload):
        self.lidar_count += 1
        if self.lidar_count % 2:
            return
        scan = downsample(unpack_points(payload), 1.5)
        if len(scan) < 50:
            return
        if not self.map.cells:
            self.map.add(scan, *self.pose)
            return
        # Constant-velocity prior: the previous registration step is the best
        # available motion model because this kinematic world reports zero
        # accelerometer and gyro values.
        prior = self.prior_delta
        tx, ty, tz, yaw, used = icp_step(scan, self.map,
                                         self.pose[0] + prior[0], self.pose[1] + prior[1],
                                         self.pose[2] + prior[2], self.pose[3] + prior[3],
                                         iterations=4)
        if used >= 30:
            step = math.hypot(tx - self.pose[0], ty - self.pose[1])
            if step < 4.0:
                self.prior_delta = (tx - self.pose[0], ty - self.pose[1],
                                    tz - self.pose[2], yaw - self.pose[3])
                self.pose = [tx, ty, tz, yaw]
                self.corrected += 1
        self.map.add(scan, *self.pose)

    def gps(self, meta):
        truth = tuple(float(v) for v in meta['position_m'])
        if not self.initialized:
            # Initial alignment on the first fix: the metric below then measures
            # drift, not the offset between the estimator and the world origin.
            self.pose = [truth[0], truth[1], truth[2], float(meta.get('yaw_rad', 0.0))]
            self.map = ScanMap()
            self.prior_delta = (0.0, 0.0, 0.0, 0.0)
            self.initialized = True
        self.track.append((float(meta['sim_time_s']), tuple(self.pose[:3]), truth))

    def report(self):
        if not self.track:
            return {}
        errors = [math.dist(est, truth) for _, est, truth in self.track]
        rmse = math.sqrt(sum(e * e for e in errors) / len(errors))
        self.out.mkdir(parents=True, exist_ok=True)
        with (self.out / 'livo_sim_trajectory.csv').open('w', encoding='utf-8') as file:
            file.write('time_s,est_x,est_y,est_z,gt_x,gt_y,gt_z,error_m\n')
            for (stamp, est, truth), error in zip(self.track, errors):
                file.write('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n'
                           % (stamp, est[0], est[1], est[2], truth[0], truth[1], truth[2], error))
        points = self.map.all_points()
        with (self.out / 'livo_sim_map.ply').open('w', encoding='utf-8') as file:
            file.write('ply\nformat ascii 1.0\nelement vertex %d\n' % len(points))
            file.write('property float x\nproperty float y\nproperty float z\nend_header\n')
            for x, y, z in points:
                file.write('%.4f %.4f %.4f\n' % (x, y, z))
        return {'poses': len(self.track), 'rmse_m': rmse,
                'max_error_m': max(errors), 'final_error_m': errors[-1],
                'icp_updates': self.corrected, 'map_points': len(points),
                'lidar_frames': self.lidar_count}


def main():
    parser = argparse.ArgumentParser(description='Simulated LiDAR-inertial estimator (FAST-LIVO2 stand-in)')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=15882)
    parser.add_argument('--seconds', type=float, default=600.0)
    parser.add_argument('--out', default='runs/livo_sim')
    options = parser.parse_args()
    out = Path(options.out)
    if not out.is_absolute():
        out = Path(__file__).resolve().parent / out
    estimator = Estimator(out)
    deadline = time.time() + options.seconds
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((options.host, options.port))
        server.listen(1)
        server.settimeout(1.0)
        print('Simulated FAST-LIVO2 estimator waiting on %s:%d' % (options.host, options.port))
        while time.time() < deadline:
            try:
                conn, address = server.accept()
            except socket.timeout:
                continue
            print('Webots connected from', address)
            with conn:
                conn.settimeout(4)
                while time.time() < deadline:
                    try:
                        meta, payload = recv_frame(conn)
                    except (OSError, EOFError):
                        print('Disconnected; finishing the run.')
                        break
                    kind = meta.get('type')
                    if kind == 'imu':
                        estimator.imu(meta)
                    elif kind == 'lidar':
                        estimator.lidar(meta, payload)
                    elif kind == 'gps':
                        estimator.gps(meta)
            if estimator.track:
                break
    summary = estimator.report()
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
