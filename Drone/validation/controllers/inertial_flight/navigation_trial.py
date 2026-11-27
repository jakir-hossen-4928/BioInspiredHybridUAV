"""LIO-SAM-driven flight: arena avoidance matrix and full farm mission.

Everything here reads its pose from pose.json, which observe_flight.py writes
only from /lio_sam/odometry/imu. Ground-truth geometry is used for scoring
(clearance, arrival), never for control.
"""
import json
import math
import os

from oak_stereo import objects
from planner import Planner

# Arena defaults; a mission JSON overrides them for the farm world.
DEFAULT = dict(goal=[10.5, 0.], boundary=14., altitude=0., truth_altitude=2., hover=9.81, mass=1.,
               mount_x=.32, step=4, steps=12500, speed=.75, gain=2.4, resolution=.25, detect_max=6.8, detect_min=.75,
               hazards=[[8, 0, .5, .6, 3.]], start=None, fixture='arena',
               obstacles=[])


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
                if k % 25 == 0:
                    candidates = []
                    if mode != 'lidar':
                        for x, y, w, disp in objects(*cams, mount_x=cfg['mount_x']):
                            if cfg['detect_min'] < x < cfg['detect_max']:
                                candidates.append(('camera', x, y, w, disp))
                    if mode != 'camera':
                        pts = sorted(((p.x, p.y) for p in sensors['lidar'].getPointCloud()
                                      if cfg['detect_min'] < p.x < cfg['detect_max']
                                      and abs(p.y) < 3 and abs(p.z + .555) < .4),
                                     key=lambda q: q[1])
                        # One candidate per lateral cluster: a thin post must not
                        # hide the wall behind it.
                        clusters = []
                        for point in pts:
                            if clusters and point[1] - clusters[-1][-1][1] < .5:
                                clusters[-1].append(point)
                            else:
                                clusters.append([point])
                        for cluster in clusters[:3]:
                            if len(cluster) < 3:
                                continue
                            ys = [q[1] for q in cluster]
                            candidates.append(('lidar', min(q[0] for q in cluster),
                                               (min(ys) + max(ys)) / 2,
                                               max(ys) - min(ys), None))
                    for candidate in candidates:
                        counts['camera' if candidate[0] == 'camera' else 'lidar'] += 1
                    if mode == 'combined' and candidates and candidates[-1][0] == 'lidar':
                        _, lx, ly, lw, _ = candidates[-1]
                        for i, (src, cx, cy, cw, cd) in enumerate(candidates[:-1]):
                            if abs(lx - cx) < 1.3 and abs(ly - cy) < .9:
                                candidates[i] = ('RGB stereo + LiDAR', lx, (ly + cy) / 2,
                                                 max(cw, lw), cd)
                                candidates.pop()
                                break
                    for src, x, y, w, disp in candidates:
                        yaw = 2 * math.atan2(msg[2][5], msg[2][6])
                        # Detected range is the obstacle's near face, so centre the
                        # box half a body depth beyond it. Centring on the range put
                        # the near face behind the UAV and the planner rejected the
                        # start cell ("no safe route").
                        sx = 1.0
                        ox = xyz[0] + math.cos(yaw) * (x + sx / 2) - math.sin(yaw) * y
                        oy = xyz[1] + math.sin(yaw) * (x + sx / 2) + math.cos(yaw) * y
                        if any(abs(ox - ob['x']) < ob['sx'] / 2 + 1.5 and
                               abs(oy - ob['y']) < ob['sy'] / 2 + 1.5 for ob in known):
                            continue  # already in the known map (near face or centre)
                        ob = dict(x=ox, y=oy, sx=sx, sy=max(1.2, w + 1.0))
                        grid = Planner(known + [ob], cfg['resolution'], cfg['boundary'], .85)
                        active = [tuple(xyz[:2])] + route
                        if all(grid.clear_segment(a, b) for a, b in zip(active, active[1:])):
                            continue
                        try:
                            path = grid.route(tuple(xyz[:2]), goal)
                        except ValueError as exc:
                            reason = 'no safe route: %s [%s at %.2f m]' % (exc, src, x)
                            break
                        known.append(ob)
                        events.append(dict(t=t, source=src, range_m=x, lateral_m=y,
                                           width_m=w, disparity_px=disp,
                                           old=active, new=path))
                        route = path[1:]
                    if reason.startswith('no safe route'):
                        break
                target = [*route[0], altitude]
                for j in range(3):
                    desired = max(-cfg['speed'], min(cfg['speed'], .95 * (target[j] - xyz[j])))
                    force[j] += max(-2. * mass, min(2. * mass,
                                                    cfg['gain'] * mass * (desired - velocity[j])))
                stable = stable + 1 if (math.dist(xyz, [*goal, altitude]) < .22 and
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
            gap = min((math.hypot(max(0., abs(pos[0] - h[0]) - h[2] - .3),
                                  max(0., abs(pos[1] - h[1]) - h[3] - .25))
                       for h in hazards if pos[2] - .1 < h[4]), default=math.inf)
            clearance = min(clearance, gap)
            contacts += int(bool(body.getContactPoints()))
            if k % 25 == 24:
                print(r.getTime(), *pos, *(last[1] if last else [float('nan')] * 3),
                      by, gap, sep=',', file=track)
            if k % 125 == 124:
                cams[0].saveImage(str(out / f'frame-{k:05}.png'), 90)
            if len(events) and not (out / 'replan.png').exists():
                r.exportImage(str(out / 'replan.png'), 95)
            if contacts or stable >= 125:
                reason = 'contact' if contacts else 'arrived'
                break
    result = dict(mode=mode, scenario=case, reason=reason, events=events,
                  observations=counts, known_obstacles=known,
                  clearance_m=clearance, contact_steps=contacts,
                  goal_error_m=math.dist(body.getPosition(), [*cfg['goal'], cfg['truth_altitude']]))
    result['passed'] = (reason == 'arrived' and bool(events) and clearance > 0
                        and not contacts and result['goal_error_m'] < .35)
    (out / 'result.json').write_text(json.dumps(result, indent=2))
