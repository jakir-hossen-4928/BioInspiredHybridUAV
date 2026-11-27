"""Render trajectory, map and route evidence as dependency-free SVG.

Usage:
    python Drone/validation/render_evidence.py <run dir> [...]
Reads ros.jsonl (odom/truth/map) and result.json (route events) and writes
trajectory.svg, map.svg and routes.svg beside them.
"""
import json
import math
import sys
from pathlib import Path

W, H, PAD = 900, 650, 40
COLORS = ['#1f77b4', '#d62728', '#2ca02c', '#ff7f0e', '#9467bd']


def svg(path, series, title):
    points = [p for _, pts in series for p in pts]
    if not points:
        return
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    scale = min((W - 2 * PAD) / max(x1 - x0, 1e-6), (H - 2 * PAD) / max(y1 - y0, 1e-6))
    def project(p):
        return (PAD + (p[0] - x0) * scale, H - PAD - (p[1] - y0) * scale)
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
           f'viewBox="0 0 {W} {H}"><rect width="{W}" height="{H}" fill="#fbfbfb"/>',
           f'<text x="{PAD}" y="24" font-family="sans-serif" font-size="16">{title}</text>']
    for i, (label, pts) in enumerate(series):
        colour = COLORS[i % len(COLORS)]
        d = ' '.join(('M' if j == 0 else 'L') + '%.1f,%.1f' % project(p)
                     for j, p in enumerate(pts))
        out.append(f'<path d="{d}" fill="none" stroke="{colour}" stroke-width="1.8"/>')
        out.append(f'<text x="{PAD + 8 + i * 210}" y="{H - 12}" font-family="sans-serif" '
                   f'font-size="12" fill="{colour}">{label}</text>')
    out.append('</svg>')
    path.write_text('\n'.join(out), encoding='utf8')


def load(directory):
    odom, truth, cloud = [], [], []
    log = directory / 'ros.jsonl'
    if not log.exists():
        return odom, truth, cloud
    for line in log.read_text(errors='replace').splitlines():
        try:
            kind, _, value = json.loads(line)
        except (ValueError, TypeError):
            continue
        if kind == 'odom':
            odom.append(value[:3])
        elif kind == 'truth':
            truth.append(value[:3])
        elif kind == 'map':
            cloud.extend(value)
    return odom, truth, cloud


def main():
    for name in sys.argv[1:]:
        directory = Path(name)
        odom, truth, cloud = load(directory)
        if odom and truth:
            # Translate odometry onto the first ground-truth sample: LIO-SAM's
            # origin is the launch pose, so this is a frame shift, not a fit.
            shift = [truth[0][i] - odom[0][i] for i in range(3)]
            svg(directory / 'trajectory.svg',
                [('Webots ground truth', [p[:2] for p in truth]),
                 ('LIO-SAM odometry', [[p[0] + shift[0], p[1] + shift[1]] for p in odom])],
                f'{directory.name}: ground truth vs LIO-SAM (odometry translated to launch pose)')
        if cloud:
            svg(directory / 'map.svg', [('LIO-SAM map points', [p[:2] for p in cloud])],
                f'{directory.name}: /lio_sam/mapping/map_global ({len(cloud)} points)')
        result = directory / 'result.json'
        if result.exists():
            events = json.loads(result.read_text()).get('events') or []
            if events:
                series = []
                for i, event in enumerate(events[:4]):
                    series.append((f'route {i + 1} before', [tuple(p) for p in event['old']]))
                    series.append((f'route {i + 1} after', [tuple(p) for p in event['new']]))
                svg(directory / 'routes.svg', series,
                    f'{directory.name}: original (blue) vs replanned (red/green/purple) routes')
        print(name, 'ok')


if __name__ == '__main__':
    main()
