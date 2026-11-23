"""Depth-based terrain assessment using ACTUAL Webots RangeFinder depth samples.

This is deliberately NOT FAST-LIVO2 / SLAM. Position comes from simulated GPS / Supervisor.
A depth camera points vertically down from a yaw-only, kinematically flown drone.
A local 3D plane is fitted to measured metric samples; landing is rejected for
unobserved, inclined, rough, or elevated surfaces.  Nothing is inferred from a
pre-authored safe/unsafe map; the terrain check consumes sensor depth values.
"""

import math
import statistics
from collections import defaultdict


def pressure_hpa_from_altitude(altitude_m):
    """ISA atmospheric pressure ESTIMATE, not a physical barometer measurement."""
    h = max(-400.0, min(10000.0, float(altitude_m)))
    return (101325.0 * (1.0 - 2.25577e-5 * h) ** 5.25588) / 100.0


def solve_three_by_three(matrix, vector):
    a = [list(row) + [rhs] for row, rhs in zip(matrix, vector)]
    for col in range(3):
        best = max(range(col, 3), key=lambda r: abs(a[r][col]))
        if abs(a[best][col]) < 1e-9:
            raise ValueError('insufficient variation for plane fit')
        a[col], a[best] = a[best], a[col]
        pivot = a[col][col]
        for j in range(col, 4):
            a[col][j] /= pivot
        for r in range(3):
            if r == col:
                continue
            ratio = a[r][col]
            for j in range(col, 4):
                a[r][j] -= ratio * a[col][j]
    return [a[i][3] for i in range(3)]


def fit_plane(points):
    """Least squares z=ax+by+c, returning slope degrees and RMS vertical residual."""
    n = len(points)
    if n < 3:
        raise ValueError('too few points')
    # Centering makes the matrix stable even when the drone is at x=-50 m.
    mean_x = sum(p[0] for p in points) / n
    mean_y = sum(p[1] for p in points) / n
    mean_z = sum(p[2] for p in points) / n
    xx = yy = xy = xz = yz = 0.0
    for x, y, z in points:
        dx, dy, dz = x - mean_x, y - mean_y, z - mean_z
        xx += dx * dx
        yy += dy * dy
        xy += dx * dy
        xz += dx * dz
        yz += dy * dz
    det = xx * yy - xy * xy
    if abs(det) < 1e-9:
        raise ValueError('degenerate footprint')
    a = (xz * yy - xy * yz) / det
    b = (yz * xx - xy * xz) / det
    c = mean_z - a * mean_x - b * mean_y
    residuals = [z - (a * x + b * y + c) for x, y, z in points]
    rms = math.sqrt(sum(r * r for r in residuals) / n)
    slope = math.degrees(math.atan(math.hypot(a, b)))
    return {'slope_deg': slope, 'roughness_m': rms,
            'max_deviation_m': max(abs(r) for r in residuals),
            'coefficients': (a, b, c)}


def analyse_depth_image(depth, width, height, horizontal_fov,
                        pose_xyz, yaw=0.0, sensor_z_offset=-0.38,
                        footprint_radius_m=1.55, min_points=12,
                        max_slope_deg=10.0, max_roughness_m=0.16,
                        max_deviation_m=0.45, max_ground_height_m=0.75,
                        sample_stride=2):
    """Evaluate landing footprint from Webots RangeFinder's *real* depth array.

    Webots returns optical-axis metric depth. The RangeFinder is downward-facing,
    with a +90-degree Y-axis rotation, and its view basis is rotated with drone
    yaw around world Z. No raycast from the known scene map is used here.
    """
    empty = {'status': 'NO_DATA', 'reason': 'depth sensor has no valid scan',
             'sample_count': 0, 'valid_ratio': 0.0, 'slope_deg': None,
             'roughness_m': None, 'max_deviation_m': None, 'median_height_m': None,
             'confidence_pct': 0, 'points': []}
    if (not depth or width < 2 or height < 2 or
            len(depth) < int(width) * int(height)):
        return empty
    tan_h = math.tan(float(horizontal_fov) / 2.0)
    tan_v = tan_h * height / width
    cx, cy, cz = (float(v) for v in pose_xyz)
    sensor_height = cz + sensor_z_offset
    yaw_cos, yaw_sin = math.cos(yaw), math.sin(yaw)
    nearby_points = []
    considered = 0
    valid = 0
    # Use a nominal ground distance to choose a consistent footprint; each
    # selected point is then transformed using its *measured* depth.
    nominal_depth = max(1.0, sensor_height)
    for j in range(0, height, sample_stride):
        for i in range(0, width, sample_stride):
            nx = (i + 0.5 - width / 2.0) / (width / 2.0)
            ny = (j + 0.5 - height / 2.0) / (height / 2.0)
            if math.hypot(nx * tan_h * nominal_depth,
                          ny * tan_v * nominal_depth) > footprint_radius_m:
                continue
            considered += 1
            r = float(depth[j * width + i])
            if not math.isfinite(r) or r < 0.09 or r > 44.5:
                continue
            valid += 1
            # Rotated camera looks down: camera image X roughly local Y;
            # camera image Y roughly local X. Sign does not affect slope.
            local_x = -ny * tan_v * r
            local_y = nx * tan_h * r
            px = cx + yaw_cos * local_x - yaw_sin * local_y
            py = cy + yaw_sin * local_x + yaw_cos * local_y
            pz = sensor_height - r
            nearby_points.append((px, py, pz))
    valid_ratio = valid / max(considered, 1)
    result = dict(empty, sample_count=valid, valid_ratio=valid_ratio,
                  points=nearby_points)
    if valid < min_points or valid_ratio < 0.70:
        result['reason'] = 'not enough valid depth returns under landing footprint'
        return result
    try:
        fit = fit_plane(nearby_points)
    except ValueError:
        result['reason'] = 'insufficient 3D footprint variation'
        return result
    median_h = statistics.median(p[2] for p in nearby_points)
    # Ground-like surfaces near farm Z=0; high flat roofs stay unsafe.
    reasons = []
    if fit['slope_deg'] > max_slope_deg:
        reasons.append('slope above limit')
    if fit['roughness_m'] > max_roughness_m:
        reasons.append('surface too rough')
    if fit['max_deviation_m'] > max_deviation_m:
        reasons.append('protrusion or uneven object in footprint')
    if median_h > max_ground_height_m or median_h < -0.45:
        reasons.append('not at expected ground level')
    status = 'UNSAFE' if reasons else 'SAFE'
    confidence = round(100 * max(0, min(1, valid_ratio)) *
                       max(0.0, 1.0 - fit['roughness_m'] / max_roughness_m) *
                       max(0.0, 1.0 - fit['slope_deg'] / (max_slope_deg * 2.0)))
    return dict(result, status=status, reason=' and '.join(reasons) if reasons else 'flat unobstructed ground-like footprint',
                slope_deg=fit['slope_deg'], roughness_m=fit['roughness_m'],
                max_deviation_m=fit['max_deviation_m'],
                median_height_m=median_h, confidence_pct=confidence)


class HeightGrid:
    """Accumulated local height observations with KNOWN simulated GPS poses.

    Not SLAM; no loop closure, no odometry estimation, no FAST-LIVO2.
    """
    def __init__(self, cell_m=1.0, max_cells=9000):
        self.cell_m = float(cell_m)
        self.max_cells = max_cells
        self.cells = {}

    def extend(self, points):
        for x, y, z in points:
            ix, iy = round(x / self.cell_m), round(y / self.cell_m)
            k = (ix, iy)
            count, mean = self.cells.get(k, (0, 0.0))
            # limit sample impact so map stays stable across scan frames
            count = min(count + 1, 30)
            mean = mean + (z - mean) / count
            if len(self.cells) < self.max_cells or k in self.cells:
                self.cells[k] = (count, mean)

    def csv_rows(self):
        for (ix, iy), (count, mean) in sorted(self.cells.items()):
            yield (round(ix * self.cell_m, 2),
                   round(iy * self.cell_m, 2), count, round(mean, 3))

    def svg(self, bounds=53.0, size=450):
        # white/grey unobserved area; color denotes measured surface height.
        from html import escape
        out = [f'<svg viewBox="0 0 {size} {size}" width="100%" xmlns="http://www.w3.org/2000/svg">',
               f'<rect width="{size}" height="{size}" fill="#15251d"/>']
        cell_pix = self.cell_m * size / (2 * bounds)
        for (ix, iy), (n, mean) in self.cells.items():
            px = (ix * self.cell_m + bounds) / (2 * bounds) * size
            py = (bounds - iy * self.cell_m) / (2 * bounds) * size
            if not (-cell_pix < px < size + cell_pix and -cell_pix < py < size + cell_pix):
                continue
            color = ('#a6e889' if -0.2 <= mean <= 0.25 else
                     '#e9c86a' if mean <= 0.75 else '#ff6a61')
            out.append(f'<rect x="{px:.1f}" y="{py:.1f}" width="{max(2, cell_pix):.2f}" height="{max(2, cell_pix):.2f}" fill="{color}"/>')
        out += ['<text x="12" y="25" fill="#e8f4ef" font-size="14">Measured terrain height cells</text>',
                '<text x="12" y="44" fill="#9fdca1" font-size="12">green: low flat-looking surface</text>',
                '<text x="12" y="61" fill="#e9c86a" font-size="12">yellow: raised surface</text>',
                '<text x="12" y="78" fill="#ff8b85" font-size="12">red: elevated surface</text>',
                '</svg>']
        return ''.join(out)
