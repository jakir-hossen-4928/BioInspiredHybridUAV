"""Hybrid A* with bounded-curvature arcs for the ground route.

Successors are short arcs with bounded curvature. Grid and heading bins
bound the search while each node retains its exact pose and incoming arc.
"""
import heapq
import itertools
import math
from dataclasses import dataclass, replace


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float
    length: float = 0.0
    curvature: float = 0.0


@dataclass(frozen=True)
class Node:
    pose: Pose
    parent: 'Node | None' = None


def arc_poses(pose, curvature, length, segments=4):
    """Poses along a constant-curvature arc, including the end point."""
    out = []
    for index in range(1, segments + 1):
        fraction = index / segments
        if abs(curvature) < 1e-9:
            out.append(Pose(pose.x + length * fraction * math.cos(pose.yaw),
                            pose.y + length * fraction * math.sin(pose.yaw),
                            pose.yaw))
            continue
        radius = 1.0 / curvature
        yaw = pose.yaw + curvature * length * fraction
        out.append(Pose(pose.x + radius * (math.sin(yaw) - math.sin(pose.yaw)),
                        pose.y - radius * (math.cos(yaw) - math.cos(pose.yaw)),
                        yaw))
    return out


def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def plan(grid, start, goal, start_yaw=0.0, goal_yaw=None,
         step=0.6, curvature=0.7, switch_cost=0.8, max_nodes=20000,
         heading_bins=24, tolerance_yaw=0.4):
    """Plan a collision-free drivable route; returns a list of Pose.

    grid must expose xy_to_cell(x, y), is_free(cell) and resolution, which is
    the same interface the grid A* and the 8-heading planner already use.
    """
    goal_xy = (float(goal[0]), float(goal[1]))
    tolerance_xy = float(grid.resolution)
    start_pose = Pose(float(start[0]), float(start[1]), float(start_yaw))
    if not grid.is_free(grid.xy_to_cell(start_pose.x, start_pose.y)):
        raise ValueError('Start lies inside an obstacle or too close to one.')
    if not grid.is_free(grid.xy_to_cell(*goal_xy)):
        raise ValueError('Goal lies inside an obstacle or too close to one.')

    bins = max(1, int(heading_bins))

    def key(pose, gear):
        cell = grid.xy_to_cell(pose.x, pose.y)
        return (*cell, int(round(_wrap(pose.yaw) / (2 * math.pi) * bins)) % bins, gear)

    def heuristic(pose):
        estimate = math.hypot(pose.x - goal_xy[0], pose.y - goal_xy[1])
        if goal_yaw is not None:
            estimate += 0.4 * abs(_wrap(goal_yaw - pose.yaw))
        return estimate

    def arrived(pose):
        if math.hypot(pose.x - goal_xy[0], pose.y - goal_xy[1]) > tolerance_xy:
            return False
        return goal_yaw is None or abs(_wrap(goal_yaw - pose.yaw)) <= tolerance_yaw

    counter = itertools.count()
    queue = [(heuristic(start_pose), 0.0, next(counter), Node(start_pose), 1)]
    best = {key(start_pose, 1): 0.0}
    segments = max(3, math.ceil(step / (grid.resolution / 4)))
    expanded = 0
    while queue and expanded < max_nodes:
        _, cost, _, node, gear = heapq.heappop(queue)
        pose = node.pose
        state = key(pose, gear)
        if cost > best.get(state, float('inf')) + 1e-9:
            continue
        expanded += 1
        if arrived(pose):
            chain = []
            while node is not None:
                chain.append(node.pose)
                node = node.parent
            return chain[::-1]
        for steer in (-curvature, 0.0, curvature):
            for direction in (1, -1):
                length = direction * step
                track = arc_poses(pose, steer, length, segments=segments)
                blocked = any(not grid.is_free(grid.xy_to_cell(p.x, p.y)) for p in track)
                if blocked:
                    continue
                child = replace(track[-1], length=length, curvature=steer)
                extra = abs(length) * (1.5 if direction < 0 else 1.0)
                if direction != gear:
                    extra += switch_cost
                extra += 0.15 * abs(steer) * abs(length)
                child_state = key(child, direction)
                new_cost = cost + extra
                if new_cost >= best.get(child_state, float('inf')):
                    continue
                best[child_state] = new_cost
                heapq.heappush(queue, (new_cost + heuristic(child), new_cost,
                                       next(counter), Node(child, node), direction))
    raise ValueError('No collision-free drivable route found.')
