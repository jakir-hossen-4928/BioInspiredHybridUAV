"""Supervisor-kinematic rolling demo for Webots quadcopter with wheel-legs.

This is a visually animated all-terrain wheel mode and retains the walking and
flight demonstrations. It does not simulate motor torque or wheel-ground grip.
Rolling uses Hybrid A* with bounded-curvature arcs over the known farm map.
"""
from collections import deque
import json
import math
import sys
from pathlib import Path

CONTROLLERS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CONTROLLERS / 'hybrid'))
from hybrid import HybridMission, BufferedKeyboard
from drone import clamp
from hybrid_astar import Pose, arc_poses, plan
from hybrid_planner import choose_safe_walk_goal
from hybrid_support import LEG_NAMES


class WheelMission(HybridMission):
    def __init__(self):
        super().__init__()
        self.rolling_height = float(self.settings.get('wheel_body_height_m', 0.60))
        self.land_height = max(self.land_height, self.rolling_height)
        self.wheel_radius = float(self.settings.get('wheel_radius_m', 0.125))
        self.rolling_speed = float(self.settings.get('wheel_drive_speed_mps', 0.92))
        self.track_width = float(self.settings.get('wheel_track_width_m', 0.91))
        if self.wheel_radius <= 0 or self.rolling_speed <= 0:
            raise ValueError('Wheel radius and ground speed must be positive')
        self.wheel_fields = {}
        self.wheel_angles = {leg: 0.0 for leg in LEG_NAMES}
        self.ground_distance = 0.0
        self.roll_progress = 0.0
        for leg in LEG_NAMES:
            node = self.robot.getFromDef('WHEEL_' + leg.upper())
            if node:
                self.wheel_fields[leg] = node.getField('rotation')
        self.state = 'TO_ROLL'
        self.paused = False
        print('[WHEEL HYBRID] tire nodes %d/4 | autonomous wheel-drive startup' %
              len(self.wheel_fields))
        print('[WHEEL HYBRID] T roll | W walk | I fly | D sensor avoidance')
        print('[WHEEL HYBRID] Ground locomotion is Supervisor-kinematic; no wheel traction model')

    def read_keys(self):
        other = []
        key = self.keyboard.getKey()
        while key != -1:
            if 0 <= key <= 255 and chr(key).upper() == 'T':
                self.request_roll()
            else:
                other.append(key)
            key = self.keyboard.getKey()
        if other:
            original = self.keyboard
            self.keyboard = BufferedKeyboard(other)
            try:
                super().read_keys()
            finally:
                self.keyboard = original

    def request_roll(self):
        if self.state in ('ROLLING', 'ROLL_HOLD', 'TO_ROLL'):
            return False
        if self.state in ('WALKING', 'WALK_HOLD', 'TO_WALK'):
            self.paused = False
            was_walking = self.state == 'WALKING'
            self.state = 'TO_ROLL'
            if was_walking and abs(self.walk_height - self.rolling_height) <= 0.003:
                self.state = 'ROLLING' if self._replan_roll() else 'ROLL_HOLD'
            print('[WHEEL HYBRID] Switched from walking to wheel drive')
            return True
        if self.state == 'LANDED':
            self.paused = False
            self.state = 'TO_ROLL'
            return True
        if self.state not in ('FLYING', 'HOVER', 'TAKEOFF', 'LANDING'):
            return False
        if super().request_walk():  # existing SAFE terrain check + mode-augmented plan
            self.state = 'TO_ROLL'
            print('[WHEEL HYBRID] SAFE descent approved for rolling mode')
            return True
        return False

    def request_walk(self):
        if self.state in ('ROLLING', 'ROLL_HOLD', 'TO_ROLL'):
            self.paused = False
            if self.state == 'ROLLING':
                self.state = 'WALKING'
                self._replan_walk()
            else:
                self.state = 'TO_WALK'
            self.walk_phase = 0.0
            print('[WHEEL HYBRID] Wheel drive changed to leg walking')
            return True
        return super().request_walk()

    def request_flight(self):
        if self.state in ('ROLLING', 'ROLL_HOLD', 'TO_ROLL'):
            self.paused = False
            self.pending_hybrid_route = None
            self.state = 'TO_FLIGHT'
            print('[WHEEL HYBRID] Wheel drive -> flight transition')
            return True
        return super().request_flight()

    def begin_roll(self):
        if not self.pending_hybrid_route:
            self.walk_goal = None
        self.pending_hybrid_route = None
        self.walk_hold_elapsed = 0.0
        if self._replan_roll():
            self.state = 'ROLLING'
        else:
            self.state = 'ROLL_HOLD'

    def _replan_roll(self):
        grid = self.make_planner()
        start = self.planar()

        def route_for_goal(goal):
            return plan(grid, start, goal, start_yaw=self.yaw)

        try:
            if self.walk_goal is None:
                self.walk_goal, self.walk_path = choose_safe_walk_goal(grid, start, route_for_goal=route_for_goal)
            else:
                self.walk_path = route_for_goal(self.walk_goal)
        except ValueError as exc:
            self.walk_path = []
            self.warning_message = str(exc)
            self.paused = True
            print('[ROLL NO PATH]', exc)
            return False
        self.walk_index = 1
        self.roll_progress = 0.0
        self.warning_message = ''
        print('[HYBRID A*] rolling poses:', len(self.walk_path), 'to', self.walk_goal)
        return True

    def _replan_walk(self):
        if self.state == 'ROLLING':
            return self._replan_roll()
        return super()._replan_walk()

    def _advance_wheels(self, forward_m, turn_rad):
        self.ground_distance += abs(forward_m)
        # Signed visual wheel rotation, including differential turning in place.
        for leg in LEG_NAMES:
            side = 1.0 if leg.endswith('l') else -1.0
            travel = forward_m - side * (turn_rad * self.track_width * 0.5)
            self.wheel_angles[leg] = (self.wheel_angles[leg] - travel / self.wheel_radius) % (2 * math.pi)

    def update_rotors(self):
        super().update_rotors()  # also keeps hip/knee neutral in rolling mode
        for leg, field in self.wheel_fields.items():
            field.setSFRotation([0, 1, 0, self.wheel_angles[leg]])

    def move_step(self):
        if self.paused:
            return
        if self.state == 'TO_ROLL':
            x, y, z = self.position
            delta = clamp(self.rolling_height - z, -0.75 * self.dt, 0.75 * self.dt)
            self._set_pose(x, y, z + delta)
            if abs(self.position[2] - self.rolling_height) <= 0.003:
                self.begin_roll()
        elif self.state == 'ROLLING':
            if self.walk_index >= len(self.walk_path):
                self.state = 'ROLL_HOLD'
                self.walk_hold_elapsed = 0.0
                self.walk_photographed = False
                print('[WHEEL HYBRID] Wheel route reached its destination')
                return
            goal = self.walk_path[self.walk_index]
            remaining = abs(goal.length) - self.roll_progress
            distance = min(self.rolling_speed * self.dt, remaining)
            signed = math.copysign(distance, goal.length)
            pose = arc_poses(Pose(*self.planar(), self.yaw), goal.curvature, signed, segments=1)[0]
            self.yaw = pose.yaw
            self._advance_wheels(signed, goal.curvature * signed)
            self._set_pose(pose.x, pose.y, self.rolling_height)
            if distance == remaining:
                self.walk_index += 1
                self.roll_progress = 0.0
            else:
                self.roll_progress += distance
        elif self.state == 'ROLL_HOLD':
            self.walk_hold_elapsed += self.dt
            if not self.walk_photographed and self.walk_hold_elapsed >= 0.75:
                self.walk_photographed = True
                self.capture('survey_camera', automatic=True, ground_inspection=True)
            if self.walk_hold_elapsed >= 2.5:
                self.request_flight()
        else:
            super().move_step()

    def write_hybrid_status(self):
        super().write_hybrid_status()
        path = self.captures_dir / self.session / 'hybrid_status.json'
        if self.state in ('ROLLING', 'ROLL_HOLD', 'TO_ROLL'):
            try:
                data = json.loads(path.read_text(encoding='utf-8'))
                data['mode'] = 'ROLL'
                data['wheels'] = {
                    'radius_m': self.wheel_radius,
                    'drive_speed_mps': self.rolling_speed,
                    'route_travel_m': round(self.ground_distance, 3),
                    'visual_angles_rad': {k: round(v, 3) for k, v in self.wheel_angles.items()},
                    'motion_model': 'kinematic animation, not motor torque / friction physics',
                }
                path.write_text(json.dumps(data, indent=2), encoding='utf-8')
            except OSError as exc:
                print('[WHEEL STATUS] Could not update status file:', exc)

    def run(self):
        while self.robot.step(self.step_ms) != -1:
            self.read_keys()
            self.sense_obstacles()
            self.update_rotors()
            self.move_step()
            now = self.robot.getTime()
            self.livo_stream.update(now, self.position, self.yaw)
            self.evaluate_terrain()
            if now - self.last_print >= float(self.settings.get('telemetry_every_s', 2)):
                self.read_lidar()
                self.update_overlay()
                self.write_sensor_dashboard()
                self.write_hybrid_status()
                from console_telemetry import emit as emit_console_telemetry
                emit_console_telemetry(self, now)
                self.robot.setLabel(10,
                    'HYBRID %s | T tires | W walk | I fly | D obstacle | %s' %
                    (self.state, self.warning_message[:48]),
                    .015, .36, .025, 0x80FFCE, .02, 'Arial')
                self.last_print = now
                print('[WHEEL HYBRID] %.1fs %s position %s' %
                      (now, self.state, [round(v, 2) for v in self.position]))


if __name__ == '__main__':
    WheelMission().run()
