"""Simple, dependency-free XY (Z-up) occupancy-grid A* planner (NOT Hybrid A* or SLAM)."""

import heapq
import math


class Planner:
    def __init__(self, obstacles, resolution=0.5, boundary=9.0, safety=0.65):
        self.obstacles = list(obstacles)
        self.resolution = float(resolution)
        self.boundary = float(boundary)
        self.safety = float(safety)
        self.n = round(2 * boundary / resolution) + 1
        self.occupied = set()
        self.rebuild()

    def xy_to_cell(self, x, z):
        return (round((x + self.boundary) / self.resolution),
                round((z + self.boundary) / self.resolution))

    def cell_to_xy(self, cell):
        return (-self.boundary + cell[0] * self.resolution,
                -self.boundary + cell[1] * self.resolution)

    def rebuild(self):
        # Include half cell width to conservatively inflate grid obstacles.
        self.occupied.clear()
        for ix in range(self.n):
            for iz in range(self.n):
                x, z = self.cell_to_xy((ix, iz))
                for ob in self.obstacles:
                    if abs(x - ob['x']) <= ob['sx'] / 2 + self.safety + self.resolution / 2 and \
                       abs(z - ob['y']) <= ob['sy'] / 2 + self.safety + self.resolution / 2:
                        self.occupied.add((ix, iz))
                        break

    def is_free(self, cell):
        ix, iz = cell
        return 0 <= ix < self.n and 0 <= iz < self.n and cell not in self.occupied

    def clear_segment(self, a, b):
        ax, az = a
        bx, bz = b
        distance = math.hypot(bx - ax, bz - az)
        count = max(1, math.ceil(distance / (self.resolution / 4)))
        return all(self.is_free(self.xy_to_cell(ax + (bx-ax)*i/count,
                                                  az + (bz-az)*i/count))
                   for i in range(count+1))

    def route(self, start, goal):
        s = self.xy_to_cell(*start)
        g = self.xy_to_cell(*goal)
        if not self.is_free(s):
            raise ValueError('Start lies inside an obstacle or too close to one.')
        if not self.is_free(g):
            raise ValueError('Goal lies inside an obstacle or too close to one.')
        moves = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
        pq = [(math.hypot(s[0]-g[0], s[1]-g[1]), 0.0, s)]
        came = {}
        cost = {s: 0.0}
        while pq:
            _, g_cost, current = heapq.heappop(pq)
            if g_cost > cost.get(current, float('inf')) + 1e-9:
                continue
            if current == g:
                raw = [current]
                while raw[-1] != s:
                    raw.append(came[raw[-1]])
                raw.reverse()
                world_pts = [self.cell_to_xy(p) for p in raw]
                # Include exact off-grid endpoints and greedily simplify with
                # clearance checks against the TRUE segment start. Previous
                # versions shortened from a rounded grid-cell start instead,
                # which could accidentally cut across a safety margin.
                raw_points = [start] + world_pts + [goal]
                simplified = [start]
                i = 0
                while i < len(raw_points) - 1:
                    j = len(raw_points) - 1
                    while j > i and not self.clear_segment(raw_points[i], raw_points[j]):
                        j -= 1
                    if j == i:
                        raise ValueError('No safe continuous segment to next cell.')
                    simplified.append(raw_points[j])
                    i = j
                return simplified
            cx, cz = current
            for dx, dz in moves:
                other = (cx+dx, cz+dz)
                if not self.is_free(other):
                    continue
                # No diagonal cutting through blocked cell corners.
                if dx and dz and (not self.is_free((cx+dx, cz)) or
                                  not self.is_free((cx, cz+dz))):
                    continue
                candidate = g_cost + math.hypot(dx, dz)
                if candidate < cost.get(other, float('inf')):
                    cost[other] = candidate
                    came[other] = current
                    heapq.heappush(pq, (candidate + math.hypot(other[0]-g[0], other[1]-g[1]),
                                        candidate, other))
        raise ValueError('No collision-free path found. Reduce clearance or change destination.')
