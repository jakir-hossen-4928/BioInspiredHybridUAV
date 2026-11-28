"""Physics-only excitation with real Webots sensors. Reference pose is logged for evaluation."""
import csv
import ctypes
from controller.wb import wb
import math
import os
import json
import tempfile
import sys
from pathlib import Path
from controller import Supervisor
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'controllers/drone'))
from fast_livo_stream import FastLivoStreamer

r = Supervisor()
body = r.getSelf()
out = Path(os.environ.get('UAV_EVIDENCE') or tempfile.mkdtemp(prefix='inertial-flight-'))
out.mkdir(parents=True, exist_ok=True)
sensors = {}
for name in ('accelerometer', 'gyro', 'inertial unit', 'lidar', 'gps'):
    sensors[name] = dev = r.getDevice(name)
    dev.enable(100 if name == 'lidar' else 4)
cams = [r.getDevice('oak_' + n) for n in ('rgb','left','right')]
for cam in cams:
    cam.enable(100)
stream = FastLivoStreamer(r, sensors, {
    'fast_livo_port': int(os.environ.get('UAV_LIO_PORT', 15882)),
    'fast_livo_lidar_interval_ms': 100,
    'native_lidar_timing': True,
}, 4)
# The fixed scanner first delivers a complete cloud at 100 ms.
stream.last_lidar = 0.
if os.environ.get('UAV_SENSOR_MODE'):
    from navigation_trial import fly
    try:
        fly(r,body,sensors,cams,stream,out,os.environ['UAV_SENSOR_MODE'],os.environ['UAV_SCENARIO'])
    except Exception:
        import traceback
        (out/'error.log').write_text(traceback.format_exc())
        r.simulationQuit(1)
        raise
    stream.close()
    r.simulationQuit(0)
    sys.exit(0)
with (out/'sensor.csv').open('w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow('t x y z roll pitch yaw ax ay az wx wy wz vx vy vz'.split())
    for k in range(5000):
        t = k * .004
        # Analytic force at the centre of mass. No pose feedback or pose writes.
        phase = 2 * math.pi * (t-3) / 12
        active = 3 <= t < 15
        accel = math.cos(phase) if active else 0
        body.addForce([.7 * accel, .18 * accel, 9.81 + .18 * accel], False)
        # R2025a's Python addTorque wrapper calls add_force. Use the native torque API.
        wb.wb_supervisor_node_add_torque(body._ref, (ctypes.c_double * 3)(.001*accel,-.0015*accel,.003*accel), 0)
        if r.step(4) == -1:
            break
        p = sensors['gps'].getValues()
        angles = sensors['inertial unit'].getRollPitchYaw()
        writer.writerow([r.getTime(), *p, *angles, *sensors['accelerometer'].getValues(),
                         *sensors['gyro'].getValues(), *body.getVelocity()[:3]])
        stream.update(r.getTime(), yaw=angles[2])
        if k == 2249:
            points = sensors['lidar'].getPointCloud()
            times = [p.time for p in points]
            audit = dict(stamp=r.getTime(), native_times=[min(times),max(times)],
                         number_of_points=len(points), rings=sorted(set(p.layer for p in points)),
                         ring_mismatches=sum(p.layer!=i//360 for i,p in enumerate(points)),
                         body_rotation=body.getOrientation(), imu_rpy=angles)
            (out/'native-scan.json').write_text(json.dumps(audit,indent=2))
        if k in (749,2249,3749):
            for cam in cams:
                cam.saveImage(str(out/f'{cam.getName()}-{k}.png'), 100)
            r.exportImage(str(out/f'world-{k}.png'), 95)
        if k % 125 == 124:
            cams[0].saveImage(str(out/f'frame-{k:04}.png'), 90)
stream.close()
r.simulationQuit(0)
