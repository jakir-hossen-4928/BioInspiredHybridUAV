"""Live, explicitly simulated Webots console telemetry.

All sensor numbers are sampled from Webots simulation devices, where available.
No invented/random sensor readings, FAST-LIVO2 odometry or SLAM poses appear here.
FAST-LIVO2 estimator processing remains an external, unverified dependency.
"""
import json
import math
from pathlib import Path

PREFIX = '[SIMULATED]'
LEGS = ('fl', 'fr', 'rl', 'rr')


def _number(value, digits=2, unit=''):
    try:
        x = float(value)
        if math.isfinite(x):
            return ('%%.%df' % digits) % x + unit
    except (ValueError, TypeError, OverflowError):
        pass
    return 'N/A'


def _vector(values, digits=2, unit=''):
    if values is None:
        return 'N/A'
    try:
        return ' / '.join(_number(x, digits, unit) for x in values)
    except (ValueError, TypeError):
        return 'N/A'


def _read(device, method, *args):
    if device is None:
        return None
    try:
        return getattr(device, method)(*args)
    except Exception:
        return None


def _device(mission, name):
    return getattr(mission, 'sensor', {}).get(name)


def _status(mission):
    state = str(getattr(mission, 'state', 'UNKNOWN'))
    if state in ('ROLLING', 'ROLL_HOLD', 'TO_ROLL'):
        return 'WHEEL', state
    if state in ('WALKING', 'WALK_HOLD', 'TO_WALK'):
        return 'WALK', state
    if state in ('TO_FLIGHT', 'FLYING', 'HOVER', 'TAKEOFF'):
        return 'FLIGHT', state
    return 'STATIONARY', state


def get_snapshot(mission, now=None):
    """Collect only actual Webots simulation observations or labeled models."""
    robot = getattr(mission, 'robot', None)
    if now is None:
        now = _read(robot, 'getTime') if robot else 0.0
    mode, state = _status(mission)
    position = getattr(mission, 'position', None)
    gps = _read(_device(mission, 'gps'), 'getValues')
    rpy = _read(_device(mission, 'inertial unit'), 'getRollPitchYaw')
    gyro = _read(_device(mission, 'gyro'), 'getValues')
    accel = _read(_device(mission, 'accelerometer'), 'getValues')
    altitude = _read(_device(mission, 'altimeter'), 'getValue')
    sonar_front = _read(_device(mission, 'ultrasonic_front'), 'getValue')
    sonar_down = _read(_device(mission, 'ultrasonic_down'), 'getValue')
    cameras = {}
    for key in ('camera', 'survey_camera', 'slam_camera'):
        cam = _device(mission, key)
        if cam is not None:
            width = _read(cam, 'getWidth')
            height = _read(cam, 'getHeight')
            cameras[key] = ([width, height] if width is not None and height is not None else None)
    stream = getattr(mission, 'livo_stream', None)
    writer = getattr(stream, 'writer', None)
    tcp_connected = bool(writer is not None and getattr(writer, 'connected', False))
    telemetry = {
        'label': 'SIMULATED',
        'time_s': now,
        'mode': mode,
        'state': state,
        'paused': bool(getattr(mission, 'paused', False)),
        'supervisor_position_xyz_m': list(position) if position is not None else None,
        'gps_xyz_m': list(gps) if gps is not None else None,
        'imu_roll_pitch_yaw_rad': list(rpy) if rpy is not None else None,
        'gyro_xyz_rad_s': list(gyro) if gyro is not None else None,
        'accel_xyz_m_s2': list(accel) if accel is not None else None,
        'altimeter_m': altitude,
        'ultrasonic_front_raw': sonar_front,
        'ultrasonic_down_raw': sonar_down,
        'lidar_nearest_m': getattr(mission, 'last_lidar', None),
        'lidar_valid_returns': getattr(mission, 'lidar_count', None),
        'cameras': cameras,
        'ros_tcp_connected': tcp_connected,
        'ros_tcp_packets_sent': getattr(writer, 'sent', 0) if writer is not None else 0,
        'ros_tcp_packets_dropped': getattr(writer, 'dropped', 0) if writer is not None else 0,
        'fast_livo2_estimator_verified': False,
        'fast_livo2_map_and_trajectory': None,
        'lidar_device_present': _device(mission, 'lidar') is not None,
        'imu_device_present': _device(mission, 'inertial unit') is not None,
        'sensor_obstacle_detection': bool(getattr(mission, 'sensor_obstacle_enabled', False)),
        'scene_obstacle_count': (len(mission.client_scene_boxes)
                                 if hasattr(mission, 'client_scene_boxes') else None),
        'sensor_obstacle': getattr(mission, 'sensor_obstacle', None),
        'planner_route_nodes': len(getattr(mission, 'walk_path', []) or []),
        'flight_astar_route_nodes': len(getattr(mission, 'waypoints', []) or []),
        'planner_type': ('Hybrid A* (bounded-curvature arcs)' if state in ('ROLLING', 'ROLL_HOLD', 'TO_ROLL')
                         else '8-heading mode-augmented grid A*' if state in ('WALKING', 'WALK_HOLD', 'TO_WALK')
                         else 'known-grid 2D A*'),
        'terrain': {k: v for k, v in (getattr(mission, 'terrain_assessment', {}) or {}).items()
                    if k in ('status', 'slope_deg', 'roughness_m', 'sample_count', 'confidence_pct')},
        'wheel_angles_rad': dict(getattr(mission, 'wheel_angles', {}) or {}),
        'wheel_route_distance_m': getattr(mission, 'ground_distance', None),
        'foot_force_xyz_n_unvalidated': {},
        'model_support_estimate_total_n': None,
        'kinematic_only': True,
    }
    feet = getattr(mission, 'walk_forces', {}) or {}
    for leg in LEGS:
        dev = feet.get(leg)
        value = _read(dev, 'getValues')
        telemetry['foot_force_xyz_n_unvalidated'][leg] = value[:3] if value else None
    if hasattr(mission, 'foot_status'):
        try:
            estimate = mission.foot_status()
            per_foot = estimate.get('estimated_support_n')
            if isinstance(per_foot, dict):
                telemetry['model_support_estimate_total_n'] = sum(float(v) for v in per_foot.values())
            elif isinstance(per_foot, (float, int)):
                telemetry['model_support_estimate_total_n'] = per_foot
        except Exception:
            pass
    return telemetry


def _degrees(values):
    try:
        return [math.degrees(float(value)) for value in values]
    except (ValueError, TypeError):
        return None


def render(snapshot):
    """Human-readable Webots console lines, each explicitly marked SIMULATED."""
    s = snapshot
    rows = [
        '----------------------------------------------------------',
        'WEBOTS LIVE TECHNICAL CONSOLE | t=%ss | %s | %s' % (
            _number(s['time_s'], 1), s['mode'], s['state']),
        'AUTONOMOUS CONTROL | %s | supervisor kinematics' % (
            'PAUSED' if s['paused'] else 'RUNNING'),
        'POSITION XYZ [m] (supervisor) | ' + _vector(s['supervisor_position_xyz_m']),
        'GPS XYZ [m] (Webots device)   | ' + _vector(s['gps_xyz_m']),
        'IMU ROLL/PITCH/YAW [deg]       | ' + _vector(_degrees(s['imu_roll_pitch_yaw_rad']), 1),
        'GYROSCOPE XYZ [rad/s]         | ' + _vector(s['gyro_xyz_rad_s'], 3),
        'ACCEL XYZ [m/s2]             | ' + _vector(s['accel_xyz_m_s2'], 2),
        'ALTIMETER [m]                | ' + _number(s['altimeter_m']),
        'LIDAR nearest [m] / returns  | %s / %s' % (
            _number(s['lidar_nearest_m']), str(s['lidar_valid_returns'])),
        'ULTRASONIC front/down [raw]  | %s / %s' % (
            _number(s['ultrasonic_front_raw']), _number(s['ultrasonic_down_raw'])),
        'CAMERAS (Webots devices)       | ' + (
            ', '.join('%s %sx%s' % (k, value[0], value[1]) if value else '%s N/A' % k
                      for k, value in s['cameras'].items()) or 'N/A'),
        'ASTAR / HYBRID PATH PLANNING  | ' + s['planner_type'],
        'PLANNED NODES flight/ground  | %d / %d' % (
            s['flight_astar_route_nodes'], s['planner_route_nodes']),
        ('OBSTACLE AVOIDANCE           | WEBOTS SCENE | nearby flight-height boxes: %d'
         % s['scene_obstacle_count'] if s['scene_obstacle_count'] is not None else
         'OBSTACLE AVOIDANCE           | %s | new object: %s' % (
            'ENABLED' if s['sensor_obstacle_detection'] else 'NOT ENABLED',
            'DETECTED' if s['sensor_obstacle'] else 'NONE')),
        'TERRAIN LANDING ASSESSMENT   | %s | slope %s deg | roughness %s m' % (
            s['terrain'].get('status', 'N/A'),
            _number(s['terrain'].get('slope_deg'), 1),
            _number(s['terrain'].get('roughness_m'), 3)),
    ]
    foot = s['foot_force_xyz_n_unvalidated']
    if any(v is not None for v in foot.values()):
        rows.append('FOOT FORCE XYZ [N] UNVALIDATED | ' + ' ; '.join(
            '%s=%s' % (leg.upper(), _vector(foot[leg], 2)) for leg in LEGS))
    else:
        rows.append('FOOT FORCE XYZ [N]           | N/A (no force-3d device data)')
    rows.append('FOOT SUPPORT MODEL [N]        | %s (model only, not measured)' %
                _number(s['model_support_estimate_total_n']))
    if s['wheel_angles_rad']:
        rows.append('WHEEL ROTATION [deg]          | ' + ' ; '.join(
            '%s=%s' % (leg.upper(), _number(math.degrees(value), 1))
            for leg, value in sorted(s['wheel_angles_rad'].items())))
        rows.append('WHEEL DRIVEN ROUTE [m]        | %s (kinematic animation)' %
                    _number(s['wheel_route_distance_m']))
    rows += [
        'ROS 2 INPUT TCP LINK          | %s | sent %d | dropped %d' % (
            'CONNECTED' if s['ros_tcp_connected'] else 'DISCONNECTED',
            s['ros_tcp_packets_sent'], s['ros_tcp_packets_dropped']),
        'FAST-LIVO2 INPUTS (simulated) | LiDAR=%s IMU=%s RGB=%s' % (
            'PRESENT' if s['lidar_device_present'] else 'MISSING',
            'PRESENT' if s['imu_device_present'] else 'MISSING',
            'PRESENT' if 'slam_camera' in s['cameras'] else 'MISSING'),
        'FAST-LIVO2 ESTIMATOR/SLAM    | NOT VERIFIED (external ROS 2 process)',
        'FAST-LIVO2 POSE / SLAM MAP   | N/A (no estimator output in Webots)',
        'NOTE | Webots sensor readings only; no physical dynamics validation',
        '----------------------------------------------------------',
    ]
    return '\n'.join('%s %s' % (PREFIX, item) for item in rows)


def emit(mission, now=None):
    """Print and atomically save the latest simulated telemetry; never stop Webots on I/O errors."""
    if not bool(getattr(mission, 'settings', {}).get('console_live_enabled', True)):
        return None
    snapshot = get_snapshot(mission, now)
    output = render(snapshot)
    print(output, flush=True)
    try:
        folder = Path(mission.captures_dir) / mission.session
        folder.mkdir(parents=True, exist_ok=True)
        latest = folder / 'CONSOLE_LIVE_STATUS.txt'
        tmp = folder / 'CONSOLE_LIVE_STATUS.tmp'
        tmp.write_text(output + '\n', encoding='utf-8')
        tmp.replace(latest)
        with (folder / 'CONSOLE_TELEMETRY.jsonl').open('a', encoding='utf-8') as f:
            f.write(json.dumps(_json_clean(snapshot), allow_nan=False) + '\n')
    except (OSError, TypeError, ValueError) as exc:
        print(PREFIX + ' CONSOLE LOG SAVE WARNING: ' + str(exc))
    return snapshot


def _json_clean(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): _json_clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_json_clean(v) for v in value]
    if isinstance(value, (bool, int, str)) or value is None:
        return value
    return str(value)
