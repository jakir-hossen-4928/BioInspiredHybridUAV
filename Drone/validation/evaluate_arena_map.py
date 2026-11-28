"""Score the last LIO-SAM global cloud against static arena geometry."""
import argparse
import json
import math
from pathlib import Path
import re

WORLD = Path(__file__).parent / 'worlds/inertial_flight.wbt'


def reference_boxes(world):
    for line in world.splitlines():
        if 'Solid {' not in line:
            continue
        match = re.search(r'translation ([^{}]+?) children .*boundingObject Box \{ size ([^}]+)', line)
        if match:
            center, size = (tuple(map(float, group.split())) for group in match.groups())
            yield center, tuple(s / 2 for s in size)


def surface_distance(point, box):
    center, half = box
    d = [abs(p - c) - h for p, c, h in zip(point, center, half)]
    return math.hypot(*(max(0., axis) for axis in d)) if max(d) > 0 else -max(d)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    run = args.run
    metadata = json.loads((run / 'run.json').read_text())
    if metadata['scenario'] != 'static' or metadata['world'] != WORLD.name:
        parser.error('a static inertial arena run is required')
    stamp = cloud = None
    with (run / 'ros.jsonl').open() as stream:
        for line in stream:
            kind, t, points = json.loads(line)
            if kind == 'map' and points:
                stamp, cloud = t, points
    if not cloud:
        raise SystemExit('No global map was published')
    boxes = list(reference_boxes(WORLD.read_text()))
    assert len(boxes) == 20
    # LIO-SAM's first map pose is the LiDAR origin. The world declares the
    # initial body z of 2 m and LiDAR offset of 0.555 m, with identity heading.
    errors = sorted(min(surface_distance((x, y, z + 2.555), box) for box in boxes)
                    for x, y, z in cloud if all(map(math.isfinite, (x, y, z))))
    if not errors:
        raise SystemExit('No finite points in the global map')
    n = len(errors)
    metric = dict(source='/lio_sam/mapping/map_global', world=WORLD.name,
                  map_stamp_s=stamp, commit=metadata['commit'], points=len(cloud),
                  finite_points=n, reference_boxes=len(boxes), origin_world_m=[0., 0., 2.555],
                  mean_surface_error_m=sum(errors) / n,
                  rmse_surface_error_m=math.sqrt(sum(e * e for e in errors) / n),
                  p95_surface_error_m=errors[math.ceil(.95 * n) - 1],
                  max_surface_error_m=errors[-1],
                  fraction_within_025m=sum(e <= .25 for e in errors) / n)
    (run / 'map-accuracy.json').write_text(json.dumps(metric, indent=2) + '\n')
    print(json.dumps(metric, indent=2))


if __name__ == '__main__':
    main()
