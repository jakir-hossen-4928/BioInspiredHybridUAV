"""Supervisor-kinematic rolling demo for Webots quadcopter with wheel-legs.

This is a visually animated all-terrain wheel mode and retains the walking and
flight demonstrations. It does not simulate motor torque or wheel-ground grip.
The ground A* state is reused for both walking and rolling (not a validated
three-mode or continuous-curvature State-Augmented Hybrid A* algorithm).
"""
from collections import deque
import json
import math
import sys
from pathlib import Path

CONTROLLERS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CONTROLLERS / 'hybrid'))
from hybrid import HybridMission, BufferedKeyboard
from drone import clamp, angle_difference
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
            self.state = 'ROLLING' if self.state == 'WALKING' else 'TO_ROLL'
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
        super().begin_walk()  # use existing 8-heading ground planner
        if self.state == 'WALKING':
            self.state = 'ROLLING'
            print('[WHEEL HYBRID] Autonomous wheel-route started: %d states' %
                  len(self.walk_path))
        elif self.state == 'WALK_HOLD':
            self.state = 'ROLL_HOLD'

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
            x, y, _ = self.position
            goal = self.walk_path[self.walk_index]
            dx, dy = goal.x - x, goal.y - y
            distance = math.hypot(dx, dy)
            if distance < 0.12:
                self.walk_index += 1
                return
            desired = math.atan2(dy, dx)
            remaining = angle_difference(desired, self.yaw)
            turn = clamp(remaining, -1.3 * self.dt, 1.3 * self.dt)
            self.yaw = angle_difference(self.yaw + turn, 0)
            forward = 0.0
            # Turn before driving to avoid visible sideways sliding.
            if abs(angle_difference(desired, self.yaw)) < 0.22:
                forward = min(self.rolling_speed * self.dt, distance)
                x += forward * math.cos(self.yaw)
                y += forward * math.sin(self.yaw)
            self._advance_wheels(forward, turn)
            self._set_pose(x, y, self.rolling_height)
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
