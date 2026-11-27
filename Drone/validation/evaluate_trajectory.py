"""Compare LIO-SAM odometry with Webots ground truth from an observe_flight ros.jsonl log.

Usage: python Drone/validation/evaluate_trajectory.py <ros.jsonl> [--json out.json]
Prints ATE (SE3-aligned, Umeyama), RPE over 1 s pairs, drift and path length.
"""
import argparse
import json
import math
from pathlib import Path


def load(path):
    odom, truth = [], []
    for line in Path(path).read_text(errors='replace').splitlines():
        try:
            record = json.loads(line)
        except ValueError:
            continue
        if not isinstance(record, list) or len(record) != 3:
            continue
        kind, stamp, value = record
        if kind == 'odom':
            odom.append((stamp, value[:3]))
        elif kind == 'truth':
            truth.append((stamp, value[:3]))
    return odom, truth


def pair(odom, truth, tolerance=.13):
    """Linear-interpolate truth at each odometry stamp; both use simulation time."""
    out, j = [], 0
    for stamp, xyz in odom:
        while j + 2 < len(truth) and truth[j + 1][0] <= stamp:
            j += 1
        if not truth or stamp < truth[0][0] or stamp > truth[-1][0]:
            continue
        a, b = truth[j], truth[min(j + 1, len(truth) - 1)]
        span = b[0] - a[0]
        if span <= 1e-9:
            continue
        ratio = min(1., max(0., (stamp - a[0]) / span))
        out.append((stamp, xyz, [a[1][i] + (b[1][i] - a[1][i]) * ratio for i in range(3)]))
    return out


def umeyama(src, dst):
    """Rigid (no scale) SE3 alignment of src onto dst; returns rotated src."""
    n = len(src)
    cs = [sum(p[i] for p in src) / n for i in range(3)]
    cd = [sum(p[i] for p in dst) / n for i in range(3)]
    cov = [[sum((p[i] - cs[i]) * (q[j] - cd[j]) for p, q in zip(src, dst)) / n
            for j in range(3)] for i in range(3)]
    # Rotation from the 3x3 covariance by polar iteration (cheap, dependency-free).
    r = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
    for _ in range(30):
        m = [[sum(cov[i][k] * r[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
        det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
               - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
               + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
        if abs(det) < 1e-12:
            break
        inv = [[(m[(j + 1) % 3][(i + 1) % 3] * m[(j + 2) % 3][(i + 2) % 3]
                 - m[(j + 1) % 3][(i + 2) % 3] * m[(j + 2) % 3][(i + 1) % 3]) / det
                for j in range(3)] for i in range(3)]
        updated = [[.5 * (r[i][j] + sum(inv[i][k] * cov[k][j] for k in range(3)))
                    for j in range(3)] for i in range(3)]
        delta = max(abs(updated[i][j] - r[i][j]) for i in range(3) for j in range(3))
        r = updated
        if delta < 1e-12:
            break
    return [[sum(r[i][k] * (p[k] - cs[k]) for k in range(3)) + cd[i] for i in range(3)]
            for p in src]


def rmse(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else float('nan')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('log', type=Path)
    parser.add_argument('--json', type=Path)
    parser.add_argument('--label', default='')
    args = parser.parse_args()
    odom, truth = load(args.log)
    matched = pair(odom, truth)
    if len(matched) < 10:
        raise SystemExit('only %d matched samples' % len(matched))
    raw = [math.dist(o, t) for _, o, t in matched]
    aligned = umeyama([o for _, o, _ in matched], [t for _, _, t in matched])
    errors = [math.dist(a, t) for a, (_, _, t) in zip(aligned, matched)]
    # RPE: pose change over a fixed 1 s interval, both streams.
    rpe = []
    for i in range(len(matched)):
        for j in range(i + 1, len(matched)):
            dt = matched[j][0] - matched[i][0]
            if dt < 1.:
                continue
            if dt > 1.05:
                break
            rpe.append(math.dist(matched[j][1], matched[i][1])
                       - math.dist(matched[j][2], matched[i][2]))
    path = sum(math.dist(matched[i + 1][2], matched[i][2]) for i in range(len(matched) - 1))
    final = math.dist(aligned[-1], matched[-1][2])
    result = dict(
        label=args.label or args.log.parent.name,
        matched_samples=len(matched),
        odometry_samples=len(odom),
        ground_truth_samples=len(truth),
        duration_s=round(matched[-1][0] - matched[0][0], 3),
        reference_path_m=round(path, 3),
        raw_position_rmse_m=round(rmse(raw), 4),
        aligned_position_rmse_m=round(rmse(errors), 4),
        aligned_position_max_m=round(max(errors), 4),
        aligned_final_error_m=round(final, 4),
        rpe_translation_rmse_m=round(rmse(rpe), 4),
        drift_percent=round(100 * final / path, 3) if path else None,
    )
    text = json.dumps(result, indent=2)
    print(text)
    if args.json:
        args.json.write_text(text + '\n')


if __name__ == '__main__':
    main()
