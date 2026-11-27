"""Forward collision-corridor measurements in the airframe (+X forward)."""
import math


def camera_hit(camera):
    width, height = camera.getWidth(), camera.getHeight()
    depth = camera.getRangeImage()
    if depth is None or len(depth) != width * height:
        return None
    focal = width / (2 * math.tan(camera.getFov() / 2))
    center_x, center_y = (width - 1) / 2, (height - 1) / 2
    hits = []
    for row in range(height):
        for col in range(1, width - 1):
            index = row * width + col
            d = depth[index]
            if not .2 <= d < 8:
                continue
            left = (center_x - col) * d / focal
            up = .05 + (center_y - row) * d / focal
            # Require adjacent pixels to agree and ignore surfaces outside the corridor.
            if (abs(left) <= .85 and abs(up) <= .55
                    and abs(depth[index - 1] - d) < .25
                    and abs(depth[index + 1] - d) < .25):
                hits.append((d + .72, left))
    return min(hits, key=lambda hit: (hit[0], abs(hit[1])), default=None)


def lidar_hit(lidar):
    hits = sorted((p.x, p.y) for p in (lidar.getPointCloud() or ())
                  if 1.0 < p.x < 8 and abs(p.y) <= .85 and abs(p.z + .555) <= .55)
    for first, second, third in zip(hits, hits[1:], hits[2:]):
        if math.dist(first, second) < .4 and math.dist(first, third) < .4:
            return first
    return None
