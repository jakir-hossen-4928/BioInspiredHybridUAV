"""Run in Webots as the wheels controller in an isolated copy of the farm.

Set UAV_AVOIDANCE_SOURCE to camera or lidar to prove each input independently.
"""
import json
import math
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'wheels'))
import wheels


source = os.environ['UAV_AVOIDANCE_SOURCE']
if source == 'camera':
    wheels.lidar_hit = lambda sensor: None
elif source == 'lidar':
    wheels.camera_hit = lambda sensor: None
else:
    raise ValueError('UAV_AVOIDANCE_SOURCE must be camera or lidar')

mission = wheels.WheelMission()
mission.stereo = [None, None, None]  # Isolate the legacy depth-proxy and LiDAR checks.
mission.livo_stream.close()
mission.show_routes = False
mission.auto_capture = False
rgb = mission.robot.getDevice('oak_rgb') if source == 'camera' else None
if source == 'camera':
    assert rgb and (rgb.getWidth(), rgb.getHeight()) == (1280, 800)
    rgb.enable(20 * mission.step_ms)
mission.state = 'FLYING'
mission.yaw = 0.
mission._set_pose(-45., -34., 13.)
goal = (-25., -34.)
mission.goal = goal
assert mission.replan()
assert mission.waypoints == [mission.planar(), mission.goal]
original = list(mission.waypoints)
box_x, box_y = -35.5, -34.
mission.world_children.importMFNodeFromString(-1, f'''
Solid {{ translation {box_x} {box_y} 13 name "unmapped obstacle"
  children [ Shape {{ appearance PBRAppearance {{ baseColor 1 .2 .05 }}
                      geometry Box {{ size 1.2 1.2 2.0 }} }} ]
  boundingObject Box {{ size 1.2 1.2 2.0 }}
}}
''')

planned = None
rgb_frame = None
clearance = math.inf
output = Path(os.environ['UAV_AVOIDANCE_OUTPUT'])
while mission.robot.step(mission.step_ms) != -1:
    if mission.robot.getTime() > 30 or mission.paused:
        output.write_text(json.dumps({'source': source, 'passed': False,
                                      'warning': mission.warning_message,
                                      'position': mission.position, 'route': mission.waypoints,
                                      'new_route': planned, 'state': mission.state,
                                      'goal': mission.goal, 'last_obstacle': mission.obstacles[-1]}), encoding='utf8')
        mission.robot.simulationQuit(1)
        break
    mission.sense_obstacles()
    if planned is None and mission.waypoints != original and mission.obstacles[-1]['name'] in ('OAK depth', 'LiDAR'):
        planned = list(mission.waypoints)
        if rgb:
            frame = output.with_suffix('.png')
            if rgb.saveImage(str(frame), 90) == 0:
                rgb_frame = frame.name
    mission.move_step()
    x, y = mission.planar()
    clearance = min(clearance, math.hypot(max(0., abs(x - box_x) - .6),
                                         max(0., abs(y - box_y) - .6)))
    if math.dist(mission.planar(), goal) < .13:
        passed = planned is not None and clearance >= 1. and (rgb is None or rgb_frame is not None)
        output.write_text(json.dumps({'source': source, 'passed': passed, 'old_route': original,
                                      'new_route': planned, 'minimum_box_clearance_m': round(clearance, 3),
                                      'goal_reached': True, 'sim_time_s': mission.robot.getTime(),
                                      'ov9782_frame': rgb_frame},
                                     indent=2), encoding='utf8')
        mission.robot.simulationQuit(0 if passed else 1)
        break
