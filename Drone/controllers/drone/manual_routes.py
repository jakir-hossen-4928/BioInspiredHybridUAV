"""Validates editable survey missions and safe grid-based A* paths.

The JSON file is edited by a standalone Tkinter app and reloaded by the Webots
Supervisor controller (key U). No ROS/PX4 or extra pip dependencies needed.

This is a known-map 2D A* planner, not on-board SLAM or Hybrid A*.
"""

import math
from planner import Planner

MAX_WAYPOINTS = 50


def _float(v, name, low, high):
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ValueError('%s must be a number' % name)
    num = float(v)
    if not math.isfinite(num) or not low <= num <= high:
        raise ValueError('%s must be between %g and %g' % (name, low, high))
    return num


def validate_manual_payload(payload, scene):
    """Return (normalized_stops, flight_config); raise ValueError on invalid data."""
    if not isinstance(payload, dict):
        raise ValueError('Manual mission file must contain a JSON object')
    raw_stops = payload.get('stops')
    if not isinstance(raw_stops, list) or not (1 <= len(raw_stops) <= MAX_WAYPOINTS):
        raise ValueError('Manual route must have 1 to %d photo stops' % MAX_WAYPOINTS)
    b = float(scene['bounds_m'])
    ids = set()
    stops = []
    for i, stop in enumerate(raw_stops):
        if not isinstance(stop, dict):
            raise ValueError('Stop %d is not an object' % (i+1))
        sid = str(stop.get('id', '')).strip()
        if not sid or len(sid)>48 or not all(c.isalnum() or c=='_' for c in sid):
            raise ValueError('Stop %d ID needs letters numbers and underscore only' % (i+1))
        if sid in ids:
            raise ValueError('Duplicate waypoint ID: '+sid)
        ids.add(sid)
        name = str(stop.get('name', sid)).strip()
        if not name or len(name)>64:
            raise ValueError('Stop %s name must be 1 to 64 characters' % sid)
        pt = stop.get('point')
        if not isinstance(pt, (tuple,list)) or len(pt)!=2:
            raise ValueError('Stop %s needs point [x,y]' % sid)
        x=_float(pt[0], '%s X' % sid, -b, b)
        y=_float(pt[1], '%s Y' % sid, -b, b)
        hover=_float(stop.get('hover_s', 4.0), '%s hover' % sid, 1.2, 20.0)
        take_photo=stop.get('take_photo', True)
        if not isinstance(take_photo,bool):
            raise ValueError('%s take_photo must be true or false' % sid)
        stops.append({'id':sid,'name':name,'point':[x,y],
                      'hover_s':hover,'take_photo':take_photo})
    opt=payload.get('flight_settings', {})
    if not isinstance(opt, dict):
        raise ValueError('flight_settings must be an object')
    config={
        'flight_speed_mps':_float(opt.get('flight_speed_mps',3.8),'Flight speed',0.5,7.0),
        'target_altitude_m':_float(opt.get('target_altitude_m',13.0),'Altitude',6.0,24.0),
        'obstacle_clearance_m':_float(opt.get('obstacle_clearance_m',1.2),'Clearance',0.4,2.5),
    }
    return stops,config


def check_safe_route(stops, scene, flight_settings, resolution=0.75):
    """Validate all legs including home; return (list_of_legs, total_distance_m).

    Validates based on *known* 2D map obstacles and safety margin. It does not
    establish 3D physical collision safety, wind resilience, or certified flight.
    """
    if not stops:
        raise ValueError('Add at least one waypoint')
    planner=Planner(scene['obstacles'], boundary=float(scene['bounds_m']),
                    resolution=resolution,
                    safety=float(flight_settings['obstacle_clearance_m']))
    positions = [tuple(scene['start'])]+[tuple(stop['point']) for stop in stops]+[tuple(scene['start'])]
    legs=[]
    distance=0.0
    for i,(a,b) in enumerate(zip(positions,positions[1:])):
        try:
            route=planner.route(a,b)
        except ValueError as e:
            label='Home' if i==len(stops) else stops[i]['name']
            raise ValueError('Route to %s is unsafe: %s' % (label,e)) from e
        legs.append(route)
        distance+=sum(math.dist(p,q) for p,q in zip(route,route[1:]))
    return legs,round(distance,2)
