"""Webots farm mission controller with optional LiDAR/IMU/camera ROS 2 streaming.

FAST-LIVO2 is a separate third-party estimator. This controller sends sensor
measurements and does not itself perform LiDAR-visual-inertial odometry.
"""

"""Farm surveying controller for Webots R2025a.

The local quadcopter moves kinematically under a Webots Supervisor. This is a
teaching simulation with metric depth-based landing assessment, not PX4,
FAST-LIVO2, Hybrid A*, or physical rotor thrust.
Known-map 2D A* flies multi-leg surveys and editable manual waypoints; each photo station stops/holds position
and captures the real simulated *downward RGB Camera* frame to disk.
"""

import csv
import html
import json
import math
import os
import sys
import time
from pathlib import Path

from controller import Supervisor

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE))
from planner import Planner
from manual_routes import validate_manual_payload, check_safe_route
from terrain_perception import (analyse_depth_image, HeightGrid,
                                pressure_hpa_from_altitude)
from fast_livo_stream import FastLivoStreamer
from console_telemetry import emit as emit_console_telemetry


def load_json(name):
    with (BASE / name).open('r', encoding='utf-8') as f:
        return json.load(f)


def clamp(x, low, high):
    return max(low, min(high, x))


def angle_difference(goal, current):
    return (goal - current + math.pi) % (2 * math.pi) - math.pi


def readable(v, digits=1):
    return ('%%.%df' % digits) % v if v is not None and math.isfinite(v) else '--'


class Mission:
    def __init__(self):
        self.robot = Supervisor()
        self.step_ms = int(self.robot.getBasicTimeStep())
        self.dt = self.step_ms / 1000.0
        self.scene = load_json('scene.json')
        self.settings = load_json('settings.json')
        self.speed = float(self.settings.get('flight_speed_mps', 3.6))
        self.altitude = float(self.settings.get('target_altitude_m', 10.5))
        self.land_height = float(self.settings.get('landing_altitude_m', .18))
        self.clearance = float(self.settings.get('obstacle_clearance_m', 1.2))
        self.hover_duration = max(1.2, float(self.settings.get('hover_duration_s', 3.0)))
        self.photo_delay = clamp(float(self.settings.get('photo_delay_s', 1.2)), 0.1,
                                 self.hover_duration - .2)
        self.auto_capture = bool(self.settings.get('auto_capture', True))
        self.capture_front_also = bool(self.settings.get('capture_front_also', True))
        self.capture_quality = int(clamp(int(self.settings.get('survey_image_quality', 97)), 1, 100))
        self.photo_format = str(self.settings.get('photo_format','jpg')).lower().lstrip('.')
        if self.photo_format not in ('jpg','png'):
            self.photo_format='jpg'
        self.base_speed = self.speed
        self.base_altitude = self.altitude
        self.base_clearance = self.clearance
        self.photo_count = 0
        self.captures_dir = BASE.parents[1] / 'captures'
        self.session = time.strftime('%Y%m%d_%H%M%S')
        self.last_photo = 'none yet'
        self.obstacles = list(self.scene['obstacles'])
        self.dynamic_on = False
        self.dynamic_obstacle = None
        self.stop_by_id = {s['id']:s for s in self.scene['photo_stops']}
        self.manual_stops = []
        self.manual_config = {}
        self._read_manual_file()   # Loads the separate route file without editing the fixed farm map
        self.mission_names = list(self.scene['missions']) + ['MANUAL_PATH']
        requested = self.settings.get('mission_mode', self.mission_names[0])
        if requested == 'MANUAL_PATH' and not self.manual_stops:
            requested = self.mission_names[0]
        if requested == 'MANUAL_PATH':
            self.apply_mode_settings(True)
        self.mode_idx = (self.mission_names.index(requested)
                         if requested in self.mission_names else 0)
        self.stop_index = 0
        self.returning = False
        self.hover_elapsed = 0.0
        self.hover_photo_saved = False
        self.paused = bool(self.settings.get('start_paused', False))
        self.show_routes = bool(self.settings.get('show_routes', True))
        self.state = 'TAKEOFF'
        self.yaw = 0.0
        self.waypoints = []
        self.waypoint_idx = 1
        self.route_error = ''
        self.path_serial = 0
        self.active_def = None
        self.overview_def = None
        self.manual_marker_defs = []
        self.last_print = -1e3
        self.last_lidar = float('inf')
        # : real depth-image perception, not a pre-authored landing-zone label.
        self.terrain_grid = HeightGrid(cell_m=1.0)
        self.terrain_assessment = {'status': 'NO_DATA', 'reason': 'awaiting first depth scan',
                                   'sample_count': 0, 'valid_ratio': 0,
                                   'slope_deg': None, 'roughness_m': None,
                                   'median_height_m': None, 'confidence_pct': 0, 'points': []}
        self.last_terrain_scan = -100.0
        self.terrain_scan_every_s = float(self.settings.get('terrain_scan_every_s', 1.25))
        self.landing_decision = 'no on-site landing requested'
        self.autonomous_safe_landing = False
        self.lidar_count = 0
        self.sensor_dashboard = BASE.parents[1] / 'dashboard.html'
        self.terrain_report_written = False


        self.node = self.robot.getSelf()
        self.position_field = self.node.getField('translation')
        self.rotation_field = self.node.getField('rotation')
        self.position = list(self.position_field.getSFVec3f())
        self.world_children = self.robot.getRoot().getField('children')
        self.dynamic_node = self.robot.getFromDef('DYNAMIC_OBSTACLE')
        self.dynamic_translation = (self.dynamic_node.getField('translation')
                                    if self.dynamic_node else None)
        self.goal_node = self.robot.getFromDef('GOAL_PAD')
        self.goal_translation = self.goal_node.getField('translation') if self.goal_node else None
        self.sensor = {}
        for name in ('gps','inertial unit','gyro','accelerometer','altimeter',
                     'camera','survey_camera','slam_camera','terrain_depth','lidar','ultrasonic_front',
                     'ultrasonic_down','foot_fl','foot_fr','foot_rl','foot_rr'):
            device = self.robot.getDevice(name)
            if not device:
                print('[SENSOR WARNING] Missing device:', name)
                continue
            period = self.step_ms
            if name in ('lidar', 'terrain_depth'):
                period = max(self.step_ms, int(self.settings.get('lidar_sampling_ms',256)))
                period = (period + self.step_ms - 1) // self.step_ms * self.step_ms
            elif name in ('camera', 'survey_camera', 'slam_camera'):
                period = 4 * self.step_ms
            device.enable(period)
            if name == 'lidar' and self.settings.get('lidar_point_cloud', False):
                try: device.enablePointCloud()
                except Exception as exc: print('[LIDAR WARNING]', exc)
            self.sensor[name] = device
        # Optional Webots sensor publisher to a separate ROS 2 / FAST-LIVO2 process.
        self.livo_stream = FastLivoStreamer(self.robot, self.sensor, self.settings, self.step_ms)
        # Each rotor is now a DEF Transform, not a physics-driven hinge joint.
        # The drone teleports kinematically every frame and resets its physics;
        # direct shape rotation is robust and visible on Webots R2025a.
        self.rotor_visuals = []
        self.rotor_angle = [0.0] * 4
        self.rotor_speed = float(self.settings.get('rotor_visual_velocity_rad_s', 21.0))
        rotor_spec = [
            ('ROTOR_FL', 1), ('ROTOR_FR', -1),
            ('ROTOR_RL', -1), ('ROTOR_RR', 1),
        ]
        for rotor_def, direction in rotor_spec:
            rotor_node = self.robot.getFromDef(rotor_def)
            if rotor_node is None:
                # Fallback when this Webots build only resolves DEF children of Robot.
                children = self.node.getField('children')
                for i in range(children.getCount()):
                    candidate = children.getMFNode(i)
                    if candidate and candidate.getDef() == rotor_def:
                        rotor_node = candidate
                        break
            if rotor_node is None:
                print('[ROTOR ERROR] Missing DEF node:', rotor_def)
                continue
            rotation_field = rotor_node.getField('rotation')
            self.rotor_visuals.append((rotation_field, direction, rotor_def))
        print('[ROTORS] Visual rotor nodes:', len(self.rotor_visuals), '/ 4')
        self.camera_resolutions={}
        for camera_name in ('survey_camera','camera'):
            cam=self.sensor.get(camera_name)
            if cam is not None:
                try:self.camera_resolutions[camera_name]=(cam.getWidth(),cam.getHeight())
                except Exception:self.camera_resolutions[camera_name]=(0,0)
        self.leds = [self.robot.getDevice('front left led'),
                     self.robot.getDevice('front right led')]
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.step_ms)
        self.set_target()
        self.replan(draw_overview=True)
        self.draw_manual_markers()
        print('\n========= AUTONOMOUS FARM SURVEY / WEBOTS R2025a =========')
        print('WORLD: spacious ~110m farm | 4 mission modes | large custom drone model | HD/4K cameras')
        print('CAMERAS:',self.camera_resolutions,'| quality=',self.capture_quality,'| format=',self.photo_format)
        print('MISSION:', self.mode_name, '| photo stops:',len(self.route_stops))
        print('P pause, M mode, N skip stop, O obstacle, R replan, V path show/hide')
        print('U RELOAD MANUAL ROUTE live from tools/path_editor.py without Webots reset')
        print('C photo down, B photo front, K PNG down, F PNG front, A auto on/off')
        print('+/- speed, H/J altitude, L land, T takeoff, G return home')
        print('Actual simulated camera images will be saved under:',self.captures_dir)
        print('Y examine landing footprint from depth | Z request guarded land-here')
        print('Sensor dashboard:', self.sensor_dashboard)
        print('FAST-LIVO2 ROS interface active only when external ROS2 bridge is running; estimator not bundled.')
        print('NOTE: Foot sensors are bumper contact only; calibrated force sensing NOT implemented.')

    def _read_manual_file(self):
        """Read an atomically saved edit and validate its known-map route.

        On invalid JSON or unsafe waypoint, the previous active route is kept.
        This check is based on a 2D static occupancy grid, not real-time LiDAR.
        """
        try:
            data = load_json('manual_mission.json')
            stops, config = validate_manual_payload(data, self.scene)
            _, distance = check_safe_route(
                stops, self.scene, config,
                resolution=float(self.settings.get('grid_resolution_m', .75))
            )
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            print('[MANUAL ROUTE ERROR] Previous route kept:', exc)
            return False
        self.manual_stops = stops
        self.manual_config = config
        print('[MANUAL ROUTE] File validated: %d stops, %.1f m total planned route' %
              (len(stops), distance))
        return True

    def apply_mode_settings(self, manual):
        if manual:
            self.speed = float(self.manual_config['flight_speed_mps'])
            self.altitude = float(self.manual_config['target_altitude_m'])
            self.clearance = float(self.manual_config['obstacle_clearance_m'])
        else:
            self.speed = self.base_speed
            self.altitude = self.base_altitude
            self.clearance = self.base_clearance

    def reload_manual_route(self):
        """Key U: load a 2D-editor route and fly its 1st leg from current position."""
        if not self._read_manual_file():
            return False
        self.mode_idx = self.mission_names.index('MANUAL_PATH')
        self.apply_mode_settings(True)
        self.stop_index = 0
        self.returning = False
        self.hover_elapsed = 0.0
        self.hover_photo_saved = False
        self._resume_flight()
        self.set_target()
        ok = self.replan(draw_overview=True)
        self.draw_manual_markers()
        print('[MANUAL ROUTE] %d stops activated; A* replan %s' %
              (len(self.manual_stops), 'OK' if ok else 'FAILED'))
        return ok

    def stop_hover_seconds(self):
        stop = self.current_stop()
        if stop is None:
            return self.hover_duration
        return max(1.2, min(20.0, float(stop.get('hover_s', self.hover_duration))))

    @property
    def mode_name(self):
        return self.mission_names[self.mode_idx]

    @property
    def route_stops(self):
        if self.mode_name == 'MANUAL_PATH':
            return self.manual_stops
        return [self.stop_by_id[i] for i in self.scene['missions'][self.mode_name]]

    def planar(self):
        return tuple(self.position[:2])

    def current_stop(self):
        return (None if self.returning or self.stop_index >= len(self.route_stops)
                else self.route_stops[self.stop_index])

    def set_target(self):
        stop = self.current_stop()
        self.goal = list(self.scene['start'] if stop is None else stop['point'])
        if self.goal_translation:
            self.goal_translation.setSFVec3f([self.goal[0], self.goal[1], .03])

    def make_planner(self):
        obs = list(self.obstacles)
        if self.dynamic_on and self.dynamic_obstacle:
            obs.append(self.dynamic_obstacle)
        return Planner(obs, resolution=float(self.settings.get('grid_resolution_m',.75)),
                       boundary=float(self.scene['bounds_m']), safety=self.clearance)

    def replan(self, draw_overview=False):
        self.planner = self.make_planner()
        try:
            self.waypoints = self.planner.route(self.planar(), self.goal)
            self.waypoint_idx = 1
            self.route_error = ''
            self.draw_paths(draw_overview=draw_overview)
            print('[A*] %d points to %s at (%.1f,%.1f)' %
                  (len(self.waypoints), 'HOME' if self.returning else self.current_stop()['name'],
                   self.goal[0], self.goal[1]))
            return True
        except ValueError as exc:
            self.route_error = str(exc)
            self.waypoints = []
            self.paused = True
            self.draw_paths(draw_overview=False)
            print('[A* NO PATH]', self.route_error)
            return False

    def remove_shape(self, def_name):
        if def_name:
            node = self.robot.getFromDef(def_name)
            if node: node.remove()

    def line_shape(self, path, name, color, z):
        if len(path) < 2:
            return None
        self.path_serial += 1
        def_name = '%s_%d' % (name, self.path_serial)
        points = ',\n'.join('%.3f %.3f %.3f' % (p[0],p[1],z) for p in path)
        indices = ' '.join(str(i) for i in range(len(path))) + ' -1'
        vrml = ('DEF %s Shape { appearance Appearance { material Material { emissiveColor %s } } '
                'geometry IndexedLineSet { coord Coordinate { point [ %s ] } coordIndex [ %s ] } }'
                % (def_name, color, points, indices))
        self.world_children.importMFNodeFromString(-1, vrml)
        return def_name

    def draw_manual_markers(self):
        """Show bright, non-collidable 3D markers for custom survey stops."""
        for name in self.manual_marker_defs:
            self.remove_shape(name)
        self.manual_marker_defs = []
        if self.mode_name != 'MANUAL_PATH':
            return
        for i, stop in enumerate(self.manual_stops):
            marker='MANUAL_WP_%02d_%d' % (i+1,self.path_serial)
            x,y=stop['point']
            vrml=('DEF %s Transform { translation %.3f %.3f %.3f children [ '
                  'Shape { appearance Appearance { material Material { '
                  'diffuseColor 1 0.65 0.1 emissiveColor 0.3 0.18 0.02 } } '
                  'geometry Sphere { radius 0.42 } } ] }' %
                  (marker,x,y,self.altitude+0.35))
            try:
                self.world_children.importMFNodeFromString(-1,vrml)
                self.manual_marker_defs.append(marker)
            except Exception as exc:
                print('[MANUAL MARKER WARNING]',exc)
        print('[MANUAL ROUTE] Visible 3D waypoint markers:',len(self.manual_marker_defs))

    def draw_paths(self, draw_overview=False):
        self.remove_shape(self.active_def)
        self.active_def = None
        if self.show_routes and len(self.waypoints) >= 2:
            self.active_def = self.line_shape(self.waypoints,'ACTIVE_LEG','1.0 .73 .15',self.altitude+.25)
        if not draw_overview:
            return
        self.remove_shape(self.overview_def)
        self.overview_def = None
        if not self.show_routes:
            return
        future = self.route_stops[self.stop_index:] if not self.returning else []
        positions=[self.planar()]+[stop['point'] for stop in future]+[self.scene['start']]
        all_points=[]
        for p0,p1 in zip(positions,positions[1:]):
            try: leg=self.planner.route(p0,p1)
            except ValueError: continue
            all_points.extend(leg if not all_points else leg[1:])
        if len(all_points)>=2:
            self.overview_def=self.line_shape(all_points,'SURVEY_ROUTE','.05 .75 .95',self.altitude+.12)

    def _resume_flight(self):
        self.state = 'FLYING' if self.state in ('FLYING', 'HOVER') else 'TAKEOFF'
        self.paused = False

    def choose_mission(self):
        next_idx = (self.mode_idx + 1) % len(self.mission_names)
        if self.mission_names[next_idx] == 'MANUAL_PATH':
            if not self._read_manual_file():
                return
        self.mode_idx = next_idx
        self.apply_mode_settings(self.mode_name == 'MANUAL_PATH')
        self.stop_index=0
        self.returning=False
        self.hover_elapsed=0
        self.hover_photo_saved=False
        self._resume_flight()
        self.set_target()
        self.replan(draw_overview=True)
        self.draw_manual_markers()
        print('[MISSION] Selected',self.mode_name, '|', len(self.route_stops),'photo stops')

    def skip_stop(self):
        if not self.returning:
            print('[MISSION] Skipped stop',self.current_stop()['name'])
            self.stop_index+=1
        if self.stop_index>=len(self.route_stops):
            self.return_home()
            return
        self._resume_flight()
        self.hover_elapsed=0
        self.hover_photo_saved=False
        self.set_target()
        self.replan(draw_overview=True)

    def return_home(self):
        self.returning=True
        self._resume_flight()
        self.hover_elapsed=0
        self.set_target()
        self.replan(draw_overview=True)
        print('[MISSION] RETURN TO HOME')

    def toggle_obstacle(self):
        if not self.dynamic_translation:
            print('[WARNING] DYNAMIC_OBSTACLE node missing')
            return
        self.dynamic_on=not self.dynamic_on
        if self.dynamic_on:
            start=self.planar()
            chosen=None
            for a,b in zip(self.waypoints[:-1],self.waypoints[1:]):
                for i in range(1,max(1,math.ceil(math.dist(a,b)/.75))+1):
                    t=i/max(1,math.ceil(math.dist(a,b)/.75))
                    pt=(a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]))
                    if math.dist(start,pt)>5 and math.dist(self.goal,pt)>3:
                        chosen=pt
                        break
                if chosen: break
            if chosen is None:
                self.dynamic_on=False
                print('[OBSTACLE] No suitable location, try during a longer leg')
                return
            self.dynamic_obstacle={'x':chosen[0],'y':chosen[1],'sx':1.55,'sy':1.55}
            self.dynamic_translation.setSFVec3f([chosen[0],chosen[1],1.35])
            print('[DECISION] New obstacle => A* route replanning')
        else:
            self.dynamic_translation.setSFVec3f([0,0,-10])
            self.dynamic_obstacle=None
            print('[DECISION] Obstacle removed => A* route replanning')
        self.replan(draw_overview=True)

    def capture(self, camera_name='survey_camera', automatic=False, file_format=None,
                *, ground_inspection=False):
        cam=self.sensor.get(camera_name)
        if cam is None:
            print('[PHOTO ERROR] Missing Webots device:',camera_name)
            return False
        stop = self.current_stop()
        if ground_inspection:
            label, stop_name = 'GROUND_INSPECTION', 'Ground inspection'
        elif stop:
            label, stop_name = stop['id'], stop['name']
        else:
            label = 'HOME' if self.returning else 'FREE_FLIGHT'
            stop_name = 'Home/free flight'
        stamp=int(self.robot.getTime()*1000)
        ext = file_format if file_format in ('jpg','png') else self.photo_format
        filename='%s_%s_%03d_%07d.%s' % (
            self.mode_name, label, self.photo_count+1, stamp, ext)
        folder=self.captures_dir/self.session
        folder.mkdir(parents=True,exist_ok=True)
        file=folder/filename
        try:
            rc=cam.saveImage(str(file),self.capture_quality)
            if rc not in (0,None):
                raise RuntimeError('Webots Camera.saveImage returned %s' % rc)
            if not file.is_file() or file.stat().st_size<=0:
                raise RuntimeError('Webots reported image save, but output file is absent/empty')
            self.photo_count+=1
            self.last_photo=filename
            manifest=folder/'photo_manifest.csv'
            exists=manifest.is_file()
            with manifest.open('a',newline='',encoding='utf-8') as f:
                writer=csv.writer(f)
                if not exists:
                    writer.writerow(['sequence','sim_time_seconds','mission','stop_id','stop_name',
                                     'camera','x_m','y_m','z_m','image_width','image_height','file','automatic'])
                writer.writerow([self.photo_count,round(self.robot.getTime(),2),
                                 self.mode_name,label,stop_name,
                                 camera_name,*[round(p,3) for p in self.position],
                                 *self.camera_resolutions.get(camera_name,(0,0)),filename,
                                 int(automatic)])
            self.write_gallery(folder)
            print('[PHOTO SAVED]',file)
            return True
        except Exception as exc:
            print('[PHOTO ERROR] Could not save Webots frame:',exc)
            return False

    def write_gallery(self, folder):
        """Self-contained local gallery; works offline without extra packages."""
        try:
            with (folder / 'photo_manifest.csv').open(newline='', encoding='utf-8') as f:
                rows = list(csv.DictReader(f))
            cards = []
            for row in rows:
                filename = html.escape(row['file'], quote=True)
                title = html.escape(row['stop_name'])
                camera = html.escape(row['camera'])
                xyz = ', '.join(row[k] for k in ('x_m', 'y_m', 'z_m'))
                cards.append('<article><a href="%s"><img src="%s" loading="lazy" alt="%s"></a>'
                             '<div class="info"><strong>%s</strong><p>%s | XYZ %s m</p>'
                             '<small>Sim time %ss</small></div></article>' %
                             (filename, filename, title, title, camera, xyz, row['sim_time_seconds']))
            css = ("body{font:16px system-ui;background:#102018;color:#e6f3ec;margin:0;}"
                   "header{padding:28px 4%;background:#183425;}h1{margin:0;}"
                   "main{display:grid;grid-template-columns:repeat(auto-fit,minmax(275px,1fr));"
                   "gap:18px;padding:25px 4%;}"
                   "article{background:#203729;border:1px solid #426b4e;border-radius:14px;overflow:hidden;}"
                   "img{width:100%;height:210px;object-fit:cover;background:#364c3f;}"
                   ".info{padding:14px;}.info p{color:#bfddc9;font-size:13px;}"
                   "small{color:#a9d0b1;}")
            content = ('<!doctype html><html lang="en"><meta charset="utf-8">'
                       '<meta name="viewport" content="width=device-width, initial-scale=1">'
                       '<title>Webots Farm Survey - Photos</title><style>%s</style>'
                       '<header><h1>Autonomous Farm Survey</h1>'
                       '<p>%d simulated RGB images | Mission %s</p></header>'
                       '<main>%s</main></html>' %
                       (css, len(rows), html.escape(self.mode_name), '\n'.join(cards)))
            (folder / 'MISSION_GALLERY.html').write_text(content, encoding='utf-8')
        except Exception as exc:
            print('[GALLERY WARNING] Could not update photo gallery:', exc)

    def evaluate_terrain(self, force=False):
        """Consume last Webots RangeFinder depth frame, build a measured height map.

        SAFE is a heuristic for the area under the *current* drone position,
        not a certified landing-zone classifier. Mission can continue if UNSAFE.
        """
        now = self.robot.getTime()
        if not force and now - self.last_terrain_scan < self.terrain_scan_every_s:
            return self.terrain_assessment
        self.last_terrain_scan = now
        sensor = self.sensor.get('terrain_depth')
        if sensor is None:
            self.terrain_assessment = dict(self.terrain_assessment, status='NO_DATA',
                                           reason='RangeFinder node missing')
            return self.terrain_assessment
        try:
            image = sensor.getRangeImage()
            report = analyse_depth_image(
                image, sensor.getWidth(), sensor.getHeight(), sensor.getFov(),
                self.position, self.yaw,
                footprint_radius_m=float(self.settings.get('landing_footprint_radius_m', 1.55)),
                max_slope_deg=float(self.settings.get('max_landing_slope_deg', 10.0)),
                max_roughness_m=float(self.settings.get('max_landing_roughness_m', 0.16)),
                max_ground_height_m=float(self.settings.get('max_landing_ground_height_m', 0.75))
            )
            self.terrain_assessment = report
            self.terrain_grid.extend(report['points'])
        except Exception as exc:
            self.terrain_assessment = dict(self.terrain_assessment, status='NO_DATA',
                                           reason='RangeFinder error ' + str(exc)[:100])
        return self.terrain_assessment

    def request_sensor_checked_landing(self):
        """Only allows landing at current position after recent measured flatness check."""
        report = self.evaluate_terrain(force=True)
        if self.position[2] < 2.5:
            self.landing_decision = 'REJECTED: require height above 2.5m for screening'
        elif self.state not in ('FLYING', 'HOVER'):
            self.landing_decision = 'REJECTED: not in flight or hover'
        elif report['status'] != 'SAFE':
            self.landing_decision = 'REJECTED: ' + report.get('reason', 'unknown terrain')
        else:
            self.autonomous_safe_landing = True
            self.landing_decision = 'APPROVED from measured depth; descending at current XY'
            self.state = 'LANDING'
            self.paused = False
        print('[SENSOR LANDING]', self.landing_decision)
        self.write_sensor_dashboard(force=True)
        return self.autonomous_safe_landing

    def sensor_snapshot(self):
        """Clearly separates measured data from derived barometric estimate."""
        gps = self.sensor.get('gps')
        imu = self.sensor.get('inertial unit')
        alt = self.sensor.get('altimeter')
        front = self.sensor.get('ultrasonic_front')
        down = self.sensor.get('ultrasonic_down')
        try: xyz = list(gps.getValues()) if gps else list(self.position)
        except Exception: xyz = list(self.position)
        try: rpy = list(imu.getRollPitchYaw()) if imu else [0.0, 0.0, self.yaw]
        except Exception: rpy = [0.0, 0.0, self.yaw]
        try: altitude = float(alt.getValue()) if alt else self.position[2]
        except Exception: altitude = self.position[2]
        def distance(dev):
            try: return float(dev.getValue()) if dev else float('nan')
            except Exception: return float('nan')
        feet=[]
        for name in ('foot_fl', 'foot_fr', 'foot_rl', 'foot_rr'):
            feet.append(distance(self.sensor.get(name)))
        return {'time_s': self.robot.getTime(), 'gps_xyz_m': xyz,
                'imu_rpy_rad': rpy, 'altimeter_m': altitude,
                'barometer_model_hpa': pressure_hpa_from_altitude(altitude),
                'ultrasonic_front': distance(front),
                'ultrasonic_down': distance(down),
                'foot_contact': feet, 'lidar_min_m': self.last_lidar,
                'lidar_valid_returns': self.lidar_count,
                'terrain': self.terrain_assessment,
                'landing_decision': self.landing_decision,
                'state': self.state, 'mission': self.mode_name}

    def write_sensor_dashboard(self, force=False):
        """Standalone HTML auto-reload for LCD 2/3; no web server needed."""
        from html import escape
        snap=self.sensor_snapshot()
        folder=self.captures_dir/self.session
        folder.mkdir(parents=True,exist_ok=True)
        assessment=snap['terrain']
        color={'SAFE':'#4ad68a','UNSAFE':'#ff7979','NO_DATA':'#e7c66d'}[assessment['status']]
        def fmt(v,unit='',digits=2):
            return ('%.*f%s' % (digits,v,unit)) if isinstance(v,(float,int)) and math.isfinite(v) else '--'
        def card(name,value,small=''):
            return '<section><span>%s</span><b>%s</b><small>%s</small></section>' % (
                escape(str(name)),escape(str(value)),escape(str(small)))
        rpy=snap['imu_rpy_rad']
        cards = [card('GPS XYZ (m)', ' / '.join(fmt(a) for a in snap['gps_xyz_m']), 'simulated local coordinates'),
                 card('IMU roll pitch yaw', ' / '.join(fmt(math.degrees(a), ' deg',1) for a in rpy), 'simulated orientation'),
                 card('LiDAR nearest', fmt(snap['lidar_min_m'],' m'), '%d finite scan returns' % snap['lidar_valid_returns']),
                 card('Ultrasonic front', fmt(snap['ultrasonic_front']), 'raw configured sensor output'),
                 card('Ultrasonic down', fmt(snap['ultrasonic_down']), 'raw configured sensor output'),
                 card('Altimeter',fmt(snap['altimeter_m'],' m'), 'simulated altitude'),
                 card('Barometric estimate',fmt(snap['barometer_model_hpa'],' hPa',1), 'derived from altitude via ISA — NOT real barometer'),
                 card('Feet contact FL FR RL RR',' / '.join(fmt(a,'',0) for a in snap['foot_contact']), 'bumper collision status — NOT force in Newtons'),
                 card('Terrain slope',fmt(assessment['slope_deg'],' deg',1), 'from measured RangeFinder depth'),
                 card('Terrain roughness',fmt(assessment['roughness_m'],' m',3), 'plane-fit RMS residual'),
                 card('Ground median height',fmt(assessment['median_height_m'],' m'), 'height relative to farm Z 0'),
                 card('Measured map cells',str(len(self.terrain_grid.cells)), 'GPS-referenced depth observations — NOT SLAM')]
        html_doc = """<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta http-equiv="refresh" content="2"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta http-equiv="Cache-Control" content="no-cache"><title>Drone | Sensors</title>
<style>body{font-family:system-ui;background:#0a1613;color:#eefbf3;margin:0}header{padding:20px 28px;background:#102a20}
h1{margin:0;font-size:24px}p{color:#b4d4c4}.layout{display:grid;grid-template-columns:minmax(270px,450px) 1fr;gap:16px;padding:20px}
section{padding:14px;background:#173429;border:1px solid #315b46;border-radius:12px;display:flex;flex-direction:column;gap:5px}
section span{font-size:13px;color:#b1cebd}section b{font-size:19px;overflow-wrap:anywhere}section small{font-size:11px;color:#a0bba9}
.metrics{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:12px}.status{font-weight:700;font-size:24px}
.map{background:#15251d;border-radius:10px;padding:10px}.note{padding:0 22px 25px;color:#b4d4c4;font-size:13px}
@media(max-width:850px){.layout{grid-template-columns:1fr}}</style></head><body>
<header><h1>Drone | Sensors and Landing</h1><p>Mission %s | State %s | Sim time %.1f s | Refresh every 2 s</p>
<div class="status" style="color:%s">LANDING FOOTPRINT %s — %s</div><p>Manual request %s</p></header>
<div class="layout"><div class="map">%s</div><div class="metrics">%s</div></div>
<div class="note">Terrain classification is a simplified heuristic from real simulated RangeFinder depth and known GPS position
No FAST LIVO2 or Livox Mid 360 driver is used
Webots flight is kinematic not physical thrust
Foot bumper signals are not calibrated forces</div></body></html>""" % (
    escape(self.mode_name),escape(self.state),snap['time_s'],color,assessment['status'],
    escape(assessment.get('reason','')), escape(self.landing_decision),
    self.terrain_grid.svg(bounds=float(self.scene['bounds_m'])), ''.join(cards))
        try:
            # os.replace works on same file volume and avoids half-written HTML.
            for destination in (self.sensor_dashboard, folder/'SENSOR_DASHBOARD.html'):
                tmp=destination.with_suffix('.tmp')
                tmp.write_text(html_doc,encoding='utf-8')
                os.replace(tmp,destination)
            csv_path=folder/'SENSOR_TELEMETRY.csv'
            first=not csv_path.exists()
            with csv_path.open('a',newline='',encoding='utf-8') as f:
                writer=csv.writer(f)
                if first: writer.writerow(['sim_time_s','state','x','y','z','imu_yaw_rad','lidar_min_m',
                                           'sonar_front_raw','sonar_down_raw','altitude_m',
                                           'barometric_model_hpa','foot_fl','foot_fr','foot_rl','foot_rr',
                                           'landing_status','slope_deg','roughness_m','depth_samples'])
                writer.writerow([round(snap['time_s'],2),snap['state'],*snap['gps_xyz_m'],
                                 snap['imu_rpy_rad'][2],snap['lidar_min_m'],
                                 snap['ultrasonic_front'],snap['ultrasonic_down'],
                                 snap['altimeter_m'],snap['barometer_model_hpa'],
                                 *snap['foot_contact'],assessment['status'],
                                 assessment['slope_deg'],assessment['roughness_m'],
                                 assessment['sample_count']])
            # Saved height map is built only from actual observed depth samples.
            with (folder/'OBSERVED_HEIGHT_MAP.csv').open('w',newline='',encoding='utf-8') as f:
                writer=csv.writer(f)
                writer.writerow(['x_m','y_m','sample_count','mean_surface_z_m'])
                writer.writerows(self.terrain_grid.csv_rows())
        except Exception as exc:
            print('[DASHBOARD WARNING]',exc)

    def read_keys(self):
        key=self.keyboard.getKey()
        while key!=-1:
            if 0<=key<=255:
                ch=chr(key).upper()
                if ch=='P':
                    self.paused=not self.paused
                    print('[CONTROL] Pause:',self.paused)
                elif ch in ('+','='):
                    self.speed=round(clamp(self.speed+.4,.5,7.0),2)
                    print('[CONTROL] Speed',self.speed)
                elif ch=='-':
                    self.speed=round(clamp(self.speed-.4,.5,7.0),2)
                    print('[CONTROL] Speed',self.speed)
                elif ch=='O': self.toggle_obstacle()
                elif ch=='M': self.choose_mission()
                elif ch=='U': self.reload_manual_route()
                elif ch=='Y':
                    result=self.evaluate_terrain(force=True)
                    print('[TERRAIN]',result['status'],result.get('reason'))
                    self.write_sensor_dashboard(force=True)
                elif ch=='Z': self.request_sensor_checked_landing()
                elif ch=='N': self.skip_stop()
                elif ch=='G': self.return_home()
                elif ch=='R': self.replan(draw_overview=True)
                elif ch=='V':
                    self.show_routes=not self.show_routes
                    self.draw_paths(draw_overview=True)
                elif ch=='C': self.capture('survey_camera',automatic=False)
                elif ch=='B': self.capture('camera',automatic=False)
                elif ch=='K': self.capture('survey_camera',automatic=False,file_format='png')
                elif ch=='F': self.capture('camera',automatic=False,file_format='png')
                elif ch=='A':
                    self.auto_capture=not self.auto_capture
                    print('[CONTROL] Automatic photos:',self.auto_capture)
                elif ch=='[':
                    self.clearance=round(clamp(self.clearance-.1,.4,2.5),2)
                    self.replan(draw_overview=True)
                elif ch==']':
                    self.clearance=round(clamp(self.clearance+.1,.4,2.5),2)
                    self.replan(draw_overview=True)
                elif ch=='H':
                    self.altitude=round(clamp(self.altitude+.5,6.0,24.0),1)
                    self.draw_paths(draw_overview=True)
                elif ch=='J':
                    self.altitude=round(clamp(self.altitude-.5,6.0,24.0),1)
                    self.draw_paths(draw_overview=True)
                elif ch=='L':
                    self.landing_decision='MANUAL OVERRIDE — no terrain safety check'
                    print('[LANDING WARNING] L overrides sensor terrain safety check')
                    self.state='LANDING'
                    self.paused=False
                elif ch=='T' and self.state in ('LANDED','LANDING'):
                    self.state='TAKEOFF'
                    self.paused=False
                    self.replan(draw_overview=True)
            key=self.keyboard.getKey()

    def move_step(self):
        """One Webots physics step. Paused/hovering does not move XY."""
        if self.paused or self.state=='LANDED':
            return
        x,y,z=self.position
        if self.state=='TAKEOFF':
            z=min(self.altitude,z+max(.7,.65*self.speed)*self.dt)
            if z>=self.altitude-.015:
                self.state='FLYING'
                print('[AUTOPILOT] TAKEOFF completed; mission route active')
        elif self.state=='LANDING':
            z=max(self.land_height,z-max(.7,.55*self.speed)*self.dt)
            if z<=self.land_height+.006:
                self.state='LANDED'
                print('[MISSION COMPLETE] Landed. Total saved photos:',self.photo_count)
                print('[MISSION COMPLETE] Captures folder:',self.captures_dir/self.session)
        elif self.state=='FLYING':
            # Smooth altitude changes while in transit.
            z+=clamp(self.altitude-z,-1.6*self.dt,1.6*self.dt)
            if self.waypoint_idx>=len(self.waypoints):
                if self.returning:
                    self.state='LANDING'
                    print('[AUTOPILOT] Home pad reached: landing')
                else:
                    self.state='HOVER'
                    self.hover_elapsed=0
                    self.hover_photo_saved=False
                    print('[HOVER] Station %d/%d: %s | hold %.1fs' %
                          (self.stop_index+1,len(self.route_stops),
                           self.current_stop()['name'],self.stop_hover_seconds()))
            else:
                tx,ty=self.waypoints[self.waypoint_idx]
                dx,dy=tx-x,ty-y
                distance=math.hypot(dx,dy)
                if distance<.12:
                    self.waypoint_idx+=1
                else:
                    stride=min(self.speed*self.dt,distance)
                    x+=stride*dx/distance
                    y+=stride*dy/distance
                    heading=math.atan2(dy,dx)
                    self.yaw+=clamp(angle_difference(heading,self.yaw),
                                    -2.4*self.dt,2.4*self.dt)
        elif self.state=='HOVER':
            # Hold XYZ still and respect each manual stop's hover/photo settings.
            self.hover_elapsed+=self.dt
            hold=self.stop_hover_seconds()
            delay=min(self.photo_delay, max(.1, hold-.2))
            stop=self.current_stop()
            photo_enabled=(stop is None or stop.get('take_photo',True))
            if (self.auto_capture and photo_enabled and not self.hover_photo_saved and
                    self.hover_elapsed>=delay):
                self.hover_photo_saved=True
                self.capture('survey_camera',automatic=True)
                if self.capture_front_also:
                    self.capture('camera',automatic=True)
            if self.hover_elapsed>=hold:
                print('[HOVER COMPLETE]', self.current_stop()['name'])
                self.stop_index+=1
                if self.stop_index>=len(self.route_stops):
                    self.return_home()
                else:
                    self.state='FLYING'
                    self.set_target()
                    self.replan(draw_overview=True)
        self.position=[x,y,z]
        self.position_field.setSFVec3f(self.position)
        self.rotation_field.setSFRotation([0,0,1,self.yaw])
        self.node.resetPhysics()

    def read_lidar(self):
        lidar=self.sensor.get('lidar')
        if not lidar:return
        try:
            values=lidar.getRangeImage()
            valid=[a for a in values if math.isfinite(a) and a>.12]
            self.lidar_count=len(valid)
            self.last_lidar=min(valid,default=float('inf'))
        except Exception as exc:
            print('[LIDAR WARNING]',exc)

    def update_overlay(self):
        gps=self.sensor.get('gps')
        imu=self.sensor.get('inertial unit')
        alt=self.sensor.get('altimeter')
        sonar=self.sensor.get('ultrasonic_front')
        coords=gps.getValues() if gps else self.position
        angles=imu.getRollPitchYaw() if imu else (0,0,self.yaw)
        alt_v=alt.getValue() if alt else self.position[2]
        front=sonar.getValue() if sonar else float('nan')
        stop=self.current_stop()
        station='HOME PAD' if self.returning else (stop['name'] if stop else 'DONE')
        progress=(len(self.route_stops) if self.returning else self.stop_index+1)
        status='PAUSED' if self.paused else self.state
        time_left=max(0,self.stop_hover_seconds()-self.hover_elapsed)
        labels=[
            ('DRONE | TERRAIN / LANDING / CAMERAS',0x66F3E1),
            ('MODE: %s   |   %s   |   %.1f m/s' % (self.mode_name,status,self.speed),0xFFFFFF),
            ('STOP: %d/%d  %s  |  HOVER %.1fs' % (progress,len(self.route_stops),station,
                                                time_left if self.state=='HOVER' else 0),0xFFFFFF),
            ('PHOTOS: %d   |   AUTO: %s   |   SAVED TO: captures/%s/' %
             (self.photo_count,'ON' if self.auto_capture else 'OFF',self.session),0xFFE196),
            ('GPS: X%.1f Y%.1f Z%.1f  |  IMU HEADING %.0f deg  |  Altimeter %sm' %
             (*coords,math.degrees(angles[2]),readable(alt_v)),0xFFFFFF),
            ('A* LEG: %d nodes  |  Clearance %.1fm  |  LiDAR %sm  |  SONAR %sm' %
             (len(self.waypoints),self.clearance,readable(self.last_lidar),readable(front)),0xFFFFFF),
            ('U manual path   Y scan ground   Z safe land   M mission   O obstacle   R replan',0xE2E7F3),
            ('C/B photos   K/F PNG   H/J altitude   +/- speed   V path visibility',0xE2E7F3),
        ]
        for i,(text,color) in enumerate(labels):
            self.robot.setLabel(i,text,.011,.016+i*.033,.027,color,.02,'Arial')
        self.robot.setLabel(8,'NO SAFE PATH: '+self.route_error if self.route_error else '',
                            .011,.290,.033,0xFF7676,.04,'Arial')
        terrain=self.terrain_assessment
        terrain_color={'SAFE':0x5EFFA7,'UNSAFE':0xFF7676,'NO_DATA':0xF6D46B}.get(terrain['status'],0xFFFFFF)
        self.robot.setLabel(9, 'TERRAIN %s | slope %s deg | roughness %s m | %s' % (
            terrain['status'],readable(terrain['slope_deg'],1),
            readable(terrain['roughness_m'],3),self.landing_decision),
            .011,.320,.025,terrain_color,.02,'Arial')

    def update_rotors(self):
        # The drone is kinematic. Render-only rotor Transform rotations avoid
        # being stopped by self.node.resetPhysics() on every motion step.
        active = not self.paused and self.state in ('TAKEOFF', 'FLYING', 'HOVER', 'LANDING')
        if active:
            for i, (field, direction, _) in enumerate(self.rotor_visuals):
                self.rotor_angle[i] = (
                    self.rotor_angle[i] + self.rotor_speed * direction * self.dt
                ) % (2.0 * math.pi)
                field.setSFRotation([0.0, 0.0, 1.0, self.rotor_angle[i]])
        for i, led in enumerate(self.leds):
            if led:
                led.set(1 if active and int(self.robot.getTime() * 2 + i) % 2 == 0 else 0)

    def run(self):
        while self.robot.step(self.step_ms)!=-1:
            self.read_keys()
            self.update_rotors()
            self.move_step()
            t=self.robot.getTime()
            self.livo_stream.update(t, self.position, self.yaw)
            self.evaluate_terrain()
            if t-self.last_print>=float(self.settings.get('telemetry_every_s',2.0)):
                self.read_lidar()
                self.update_overlay()
                self.write_sensor_dashboard()
                emit_console_telemetry(self, t)
                self.last_print=t
                print('[SURVEY] t=%6.1fs state=%-7s pos=(%.1f,%.1f,%.1f) '
                      'stop=%d/%d photos=%d' %
                      (t,self.state,*self.position,self.stop_index+1,
                       len(self.route_stops),self.photo_count))


if __name__=='__main__':
    Mission().run()
