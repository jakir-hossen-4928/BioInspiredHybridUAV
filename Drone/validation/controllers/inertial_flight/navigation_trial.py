"""LIO-SAM-driven flight: arena avoidance matrix and full farm mission.

Everything here reads its pose from pose.json, which observe_flight.py writes
only from /lio_sam/odometry/imu. Ground-truth geometry is used for scoring
(clearance, arrival), never for control.
"""
import json
import math
import os
from statistics import median

from oak_stereo import objects
from planner import Planner

# Arena defaults; a mission JSON overrides them for the farm world.
DEFAULT = dict(goal=[10.5, 0.], boundary=14., altitude=0., truth_altitude=2., hover=9.81, mass=1.,
               mount_x=.32, step=4, steps=12500, speed=.75, gain=2.4, resolution=.25, detect_max=6.8, detect_min=.75,
               hazards=[[8, 0, .5, .6, 3.]], start=None, fixture='arena',
               obstacles=[], body_half=[.3, .25, .1], obstacle_speed=.75)


class ObstacleTracks:
    """Keep a swept footprint through missed frames and estimate near-term motion."""

    def __init__(self):
        self.samples = []

    def observe(self, observations, stamp):
        matched = set()
        for ob in observations:
            choices = [(math.hypot(ob['x'] - track[-1][1]['x'],
                                   ob['y'] - track[-1][1]['y']), i)
                       for i, track in enumerate(self.samples) if i not in matched]
            distance, i = min(choices, default=(math.inf, -1))
            if distance < 2.5:
                self.samples[i].append((stamp, ob))
                self.samples[i] = self.samples[i][-30:]
            else:
                i = len(self.samples)
                self.samples.append([(stamp, ob)])
            matched.add(i)

    def footprints(self, now, horizon, max_speed):
        footprints = []
        for track in self.samples:
            stamp, current = track[-1]
            recent = track[-5:]
            x = median(ob['x'] for _, ob in recent)
            y = median(ob['y'] for _, ob in recent)
            old_stamp, old = track[0]
            duration = max(.1, stamp - old_stamp)
            vy = max(-max_speed, min(max_speed, (y - old['y']) / duration))
            # Cover previously observed reversal and the unobserved travel
            # during the next brake, including any camera dropout.
            # shortcut: one metre of unseen motion, upgrade with peripheral coverage.
            unseen = min(1., max_speed * (max(0., now - stamp) + horizon))
            lower = min(ob['y'] - ob['sy'] / 2 for _, ob in track)
            upper = max(ob['y'] + ob['sy'] / 2 for _, ob in track)
            predicted = y + vy * horizon
            lower = min(lower, predicted - current['sy'] / 2) - unseen
            upper = max(upper, predicted + current['sy'] / 2) + unseen
            footprints.append(dict(x=x, y=(lower + upper) / 2, sx=current['sx'],
                                   sy=upper - lower, source='sensor'))
        return footprints


def braking_segment(position, velocity, latency, gain):
    speed = math.hypot(*velocity)
    if not speed:
        return position
    # Integrate the saturated proportional brake to its exponential tail.
    crossover = min(speed, 2. / gain)
    distance = speed * latency + (speed**2 - crossover**2) / 4 + crossover / gain
    return tuple(p + v * distance / speed for p, v in zip(position, velocity))


def cluster_2d(points, cell=.7):
    """Group body-frame points by 2D proximity (union of neighbouring cells)."""
    buckets = {}
    for p in points:
        buckets.setdefault((int(p[0] // cell), int(p[1] // cell)), []).append(p)
    seen, groups = set(), []
    for key in buckets:
        if key in seen:
            continue
        seen.add(key)
        stack, group = [key], []
        while stack:
            k = stack.pop()
            group.extend(buckets[k])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    other = (k[0] + dx, k[1] + dy)
                    if other in buckets and other not in seen:
                        seen.add(other)
                        stack.append(other)
        groups.append(group)
    return groups


def fly(r, body, sensors, cams, stream, out, mode, case):
    cfg = dict(DEFAULT)
    path = os.environ.get('UAV_MISSION_JSON')
    if path:
        cfg.update(json.loads(open(path, encoding='utf8').read()))
    # LIO-SAM's odom frame is anchored at the launch pose (yaw = body yaw at
    # launch, which is 0 in both worlds), so every world coordinate the
    # controller reasons about must be shifted by the launch position.
    origin = tuple(cfg['start'][:2]) if cfg['start'] else (0., 0.)
    goal = (cfg['goal'][0] - origin[0], cfg['goal'][1] - origin[1])
    route = [(0., 0.), goal]
    known = [dict(ob, x=ob['x'] - origin[0], y=ob['y'] - origin[1])
             for ob in cfg['obstacles']]
    surveyed = known
    tracks = ObstacleTracks()
    hazards = cfg['hazards']
    altitude, mass, step = cfg['altitude'], cfg['mass'], cfg['step']
    box = None
    if cfg['fixture'] == 'arena':
        box = r.getFromDef('OBSTACLE').getField('translation')
    if case == 'multiple':
        r.getRoot().getField('children').importMFNodeFromString(-1,
            'DEF SECOND Solid { translation 5 -1.4 2 children [ Shape { appearance PBRAppearance { baseColor 1 .05 .02 } geometry Box { size 1 1.2 2 } } ] boundingObject Box { size 1 1.2 2 } }')
        hazards = hazards + [[5, -1.4, .5, .6, 3.]]
    if cfg['start']:
        body.getField('translation').setSFVec3f(cfg['start'])
    if cfg.get('obstacle_at'):
        r.getFromDef('DYNAMIC_OBSTACLE').getField('translation').setSFVec3f(cfg['obstacle_at'])
    events = []
    safety_events = []
    hold = False
    velocity = [0.] * 3
    clearance = math.inf
    contacts = 0
    last = None
    stable = 0
    bad_velocity = 0
    reason = 'timeout'
    counts = {'camera': 0, 'lidar': 0}
    with (out / 'track.csv').open('w') as track:
        print('t,x,y,z,odom_x,odom_y,odom_z,box_y,clearance', file=track)
        for k in range(cfg['steps']):
            t = k * step / 1000.
            by = 0.
            if case == 'moving' and box is not None:
                by = .8 * math.sin(.7 * t)
                box.setSFVec3f([8, by, 2])
            fresh = False
            try:
                msg = json.loads((out / 'pose.json').read_text())
                stamp = msg[1]
                xyz = msg[2][:3]
                fresh = 0 <= t - stamp <= .65
            except (FileNotFoundError, PermissionError, ValueError):
                pass
            force = [0., 0., cfg['hover']]
            if fresh:
                if len(msg[2]) >= 10:
                    v = msg[2][7:10]
                    if math.sqrt(sum(c * c for c in v)) < 6.:
                        velocity, bad_velocity = v, 0
                    else:
                        # Never chase a diverged estimator: hold and then abort.
                        bad_velocity += 1
                        if bad_velocity >= 25:
                            reason = 'estimator diverged'
                            break
                elif last and stamp > last[0]:
                    velocity = [(u - v) / (stamp - last[0]) for u, v in zip(xyz, last[1])]
                last = (stamp, xyz)
                if len(route) > 1 and math.dist(xyz[:2], route[0]) < .24:
                    route.pop(0)
                if k % max(1, round(100 / step)) == 0:
                    candidates = []
                    if mode != 'lidar':
                        for x, y, w, disp in objects(*cams, mount_x=cfg['mount_x']):
                            if cfg['detect_min'] < x < cfg['detect_max']:
                                candidates.append(('camera', x, y, w, disp, None))
                    if mode != 'camera':
                        pts = [(p.x, p.y) for p in sensors['lidar'].getPointCloud()
                               if cfg['detect_min'] < p.x < cfg['detect_max']
                               and abs(p.y) < 3 and abs(p.z + .555) < .4]
                        # Cluster in 2D, not by lateral gap alone: a near box and
                        # the far wall behind it must not merge into one 6 m wall.
                        for group in sorted(cluster_2d(pts), key=lambda g: min(q[0] for q in g))[:3]:
                            if len(group) < 3:
                                continue
                            xs = [q[0] for q in group]
                            ys = [q[1] for q in group]
                            candidates.append(('lidar', min(xs), (min(ys) + max(ys)) / 2,
                                               max(ys) - min(ys), None, max(xs) - min(xs)))
                    if mode == 'combined':
                        lidar = [c for c in candidates if c[0] == 'lidar']
                        for cam in (c for c in candidates if c[0] == 'camera'):
                            for i, lid in enumerate(lidar):
                                if abs(lid[1] - cam[1]) < 1.3 and abs(lid[2] - cam[2]) < .9:
                                    lidar[i] = ('RGB stereo + LiDAR', lid[1],
                                                (lid[2] + cam[2]) / 2,
                                                max(lid[3], cam[3]), cam[4], lid[5])
                                    break
                            else:
                                lidar.append(cam)
                        candidates = lidar
                    observations = []
                    for src, x, y, w, disp, depth in candidates:
                        counts['camera' if src == 'camera' else 'lidar'] += 1
                        qx, qy, qz, qw = msg[2][3:7]
                        yaw = math.atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz))
                        # The detected range is the obstacle's near face, so the
                        # box centre sits half a depth beyond it. Centring on the
                        # range put the near face behind the UAV and the planner
                        # rejected the start cell ("no safe route").
                        # Depth is capped: a single view cannot measure it, and an
                        # inflated cluster (a ring of stray returns) would push
                        # the box centre onto the destination.
                        sx = min(1.5, max(.6, depth or 1.))
                        ox = xyz[0] + math.cos(yaw) * (x + sx / 2) - math.sin(yaw) * y
                        oy = xyz[1] + math.sin(yaw) * (x + sx / 2) + math.cos(yaw) * y
                        ob = dict(x=ox, y=oy, sx=sx, sy=max(1.2, w + .6), source='sensor')
                        observations.append(ob)
                    tracks.observe(observations, t)
                    speed = math.hypot(*velocity[:2])
                    latency = t - stamp + .2  # camera period and controller polling period
                    horizon = latency + speed / 2. + 1. / cfg['gain']
                    known = surveyed + tracks.footprints(t, horizon, cfg['obstacle_speed'])
                    grid = Planner(known, cfg['resolution'], cfg['boundary'], .85)
                    start = tuple(xyz[:2])
                    active = [start] + route
                    stop = braking_segment(start, velocity[:2], latency, cfg['gain'])
                    hold_reason = None
                    if not grid.clear_segment(start, stop):
                        hold_reason = 'stopping corridor occupied'
                    elif not all(grid.clear_segment(a, b) for a, b in zip(active, active[1:])):
                        try:
                            path = grid.route(start, goal)
                        except ValueError as exc:
                            hold_reason = str(exc)
                        else:
                            events.append(dict(t=t, source=mode, observations=candidates,
                                               old=active, new=path))
                            route = path[1:]
                    if hold != bool(hold_reason):
                        safety_events.append(dict(t=t, reason=hold_reason, pose_age_s=t-stamp,
                                                  stopping_distance_m=math.dist(start, stop)))
                    hold = bool(hold_reason)
                target = [*route[0], altitude]
                for j in range(3):
                    desired = max(-cfg['speed'], min(cfg['speed'], .95 * (target[j] - xyz[j])))
                    if hold and j < 2:
                        desired = 0.
                    force[j] += max(-2. * mass, min(2. * mass,
                                                    cfg['gain'] * mass * (desired - velocity[j])))
                stable = stable + 1 if (not hold and math.dist(xyz, [*goal, altitude]) < .22 and
                                        math.hypot(*velocity[:2]) < .25) else 0
            elif last and t - last[0] > 1.5:
                reason = 'stale odometry'
                break
            body.addForce(force, False)
            if r.step(step) == -1:
                break
            stream.update(r.getTime(), yaw=sensors['inertial unit'].getRollPitchYaw()[2])
            # Truth is read only after issuing the force, for scoring.
            pos = body.getPosition()
            if case == 'moving' and box is not None:
                hazards = [h for h in hazards if h[0] != 8] + [[8, by, .5, .6, 3.]]
            hx, hy, hz = cfg['body_half']
            gap = min((math.hypot(max(0., abs(pos[0] - h[0]) - h[2] - hx),
                                  max(0., abs(pos[1] - h[1]) - h[3] - hy))
                       for h in hazards if pos[2] - hz < h[4]), default=math.inf)
            clearance = min(clearance, gap)
            contacts += int(bool(body.getContactPoints()))
            if k % 25 == 24:
                print(r.getTime(), *pos, *(last[1] if last else [float('nan')] * 3),
                      by, gap, sep=',', file=track)
            if k % 125 == 124:
                cams[0].saveImage(str(out / f'frame-{k:05}.png'), 90)
                # Stereo pair kept for offline depth checks (see test_stereo_depth).
                for name, cam in zip(('left', 'right'), cams[1:]):
                    cam.saveImage(str(out / f'{name}-{k:05}.png'), 90)
            if len(events) and not (out / 'replan.png').exists():
                r.exportImage(str(out / 'replan.png'), 95)
            if contacts or stable >= 125:
                reason = 'contact' if contacts else 'arrived'
                break
    result = dict(mode=mode, scenario=case, reason=reason, events=events,
                  safety_events=safety_events,
                  observations=counts, known_obstacles=known,
                  clearance_m=clearance, contact_steps=contacts,
                  goal_error_m=math.dist(body.getPosition(), [*cfg['goal'], cfg['truth_altitude']]))
    result['passed'] = (reason == 'arrived' and bool(events) and clearance > 0
                        and not contacts and result['goal_error_m'] < .35)
    (out / 'result.json').write_text(json.dumps(result, indent=2))
