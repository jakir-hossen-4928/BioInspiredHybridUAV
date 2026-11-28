"""Build the full-farm mission file and the farm-flight world copy.

The copy keeps every texture path valid (same worlds/ folder) and only swaps
the DRONE controller. Regenerate with:
    python Drone/validation/farm_mission.py --output <evidence dir>
"""
import argparse
import json
import re
from pathlib import Path

DRONE = Path(__file__).resolve().parents[1]
OBSTACLE = [-30.5, -30.3, 4.5]  # unknown mid-air obstacle, ~25 m along the route
GOAL = [40.0, 33.0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    scene = json.loads((DRONE / 'controllers/drone/scene.json').read_text(encoding='utf8'))
    mission = dict(
        goal=GOAL, boundary=115.0, altitude=0.0, truth_altitude=4.5,
        hover=3.9 * 9.81, mass=3.9,   # real gravity: 3.9 kg airframe hovers on 38.3 N
        mount_x=.72, step=8, steps=24000, speed=1.5, gain=2.4,
        resolution=1.0, detect_max=10.0, detect_min=1.1, body_half=[.6075, .3825, .372],
        start=[-49.0, -47.0, 4.5], fixture=None, obstacle_at=OBSTACLE,
        obstacles=[dict(x=o['x'], y=o['y'], sx=o['sx'], sy=o['sy'])
                   for o in scene['obstacles'] if o['height'] >= 4.5 - .372 - .85],
        hazards=[[o['x'], o['y'], o['sx'] / 2, o['sy'] / 2, o['height']]
                 for o in scene['obstacles']] + [OBSTACLE[:2] + [.775, .775, 5.85]],
    )
    (args.output / 'mission.json').write_text(json.dumps(mission, indent=2))
    source = (DRONE / 'worlds/hybrid_wheels_ros2.wbt').read_text(encoding='utf8')
    target = source.replace('supervisor TRUE controller "wheels"',
                            'supervisor TRUE controller "wheels" controllerArgs [ "--lio-sam-flight" ]', 1)
    if target == source:
        raise SystemExit('DRONE controller line not found; world layout changed')
    # Accept the mission under real gravity instead of the demo's zero-g world.
    target, changed = re.subn(r'(WorldInfo \{[^}]*?gravity\s+)0(\s)', r'\g<1>9.81\2', target, count=1)
    if not changed:
        raise SystemExit('WorldInfo gravity not found; world layout changed')
    (DRONE / 'worlds/farm_flight.wbt').write_text(target, encoding='utf8')
    print(json.dumps(dict(mission=str(args.output / 'mission.json'),
                          world=str(DRONE / 'worlds/farm_flight.wbt')), indent=2))


if __name__ == '__main__':
    main()
