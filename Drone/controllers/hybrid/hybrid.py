"""Webots aerial-ground navigation demonstration for a quadcopter with four legs.

Educational scope: Supervisor-driven kinematic motion, visible gait and safe
mode changes, 8-heading walking search, and live sonar-based replanning.
Not a validated legged dynamics controller or an original research Hybrid A*.
"""
from collections import deque
import csv
import json
import math
import sys
from pathlib import Path

CONTROLLERS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CONTROLLERS / 'drone'))
from drone import Mission, clamp, angle_difference
from hybrid_planner import HybridStatePlanner, choose_safe_walk_goal
from hybrid_support import LEG_NAMES, gait_angles, force_estimate, projected_obstacle


class BufferedKeyboard:
    def __init__(self, events):
        self.events=deque(events)
    def getKey(self):
        return self.events.popleft() if self.events else -1


class HybridMission(Mission):
    def __init__(self):
        super().__init__()
        self.walk_height = float(self.settings.get('walk_body_height_m',0.60))
        self.land_height = max(self.land_height, self.walk_height)
        self.walk_speed = float(self.settings.get('walking_speed_mps',0.65))
        self.walk_phase = 0.0
        self.walk_index = 1
        self.walk_path = []
        self.walk_goal = None
        self.pending_hybrid_route = None
        self.walk_hold_elapsed = 0.0
        self.walk_photographed = False
        self.sensor_obstacle_stop_m = float(self.settings.get('sensor_obstacle_stop_m', 1.8))
        self.sensor_obstacle_enabled = True
        self.sensor_obstacle = None
        self.sensor_obstacle_time = -100.0
        self.last_sensor_check = -100.0
        self.warning_message = ''
        self.leg_fields = {}
        self.knee_fields = {}
        self.walk_forces = {}
        for leg in LEG_NAMES:
            hip=self.robot.getFromDef('LEG_%s_HIP'%leg.upper())
            knee=self.robot.getFromDef('LEG_%s_KNEE'%leg.upper())
            if hip:self.leg_fields[leg]=hip.getField('rotation')
            if knee:self.knee_fields[leg]=knee.getField('rotation')
            dev=self.robot.getDevice('walk_force_'+leg)
            if dev:
                dev.enable(self.step_ms)
                self.walk_forces[leg]=dev
        if bool(self.settings.get('hybrid_demo_start_walk', True)):
            # The designated home pad is pre-marked safe for the initial walking demo.
            # Later flight-to-walk transitions require an actual depth check.
            self.state='TO_WALK'
            self.paused=False
        print('[HYBRID] Legs %d/4 | foot force devices %d/4' %
              (len(self.leg_fields),len(self.walk_forces)))
        print('[HYBRID] W descend & walk | I return to flight | D obstacle sensing toggle')
        print('[HYBRID] Foot force values are raw simulation readings; estimated support separate')

    def read_keys(self):
        other=[]
        key=self.keyboard.getKey()
        while key!=-1:
            if 0<=key<=255 and chr(key).upper() == 'W':
                self.request_walk()
            elif 0<=key<=255 and chr(key).upper() == 'I':
                self.request_flight()
            elif 0<=key<=255 and chr(key).upper() == 'D':
                self.sensor_obstacle_enabled = not self.sensor_obstacle_enabled
                print('[SENSOR AVOIDANCE]',self.sensor_obstacle_enabled)
            else:other.append(key)
            key=self.keyboard.getKey()
        if other:
            real=self.keyboard
            self.keyboard=BufferedKeyboard(other)
            try:super().read_keys()
            finally:self.keyboard=real

    def request_walk(self):
        if self.state in ('WALKING','TO_WALK','WALK_HOLD'):
            print('[HYBRID] Already in walking mode')
            return False
        if self.state == 'LANDED':
            self.paused=False
            self.state='TO_WALK'
            print('[HYBRID] Stand and transition from stationary mode to walking')
            return True
        if self.state not in ('FLYING','HOVER','TAKEOFF','LANDING'):
            print('[HYBRID] Cannot descend from state',self.state)
            return False
        report=self.evaluate_terrain(force=True)
        if report.get('status')!='SAFE':
            self.warning_message='Walking transition rejected: '+report.get('reason','terrain is not SAFE')
            print('[HYBRID]',self.warning_message)
            return False
        try:
            grid=self.make_planner()
            goal,_=choose_safe_walk_goal(grid,self.planar())
            self.pending_hybrid_route=HybridStatePlanner(
                grid,transition_zones=[self.planar()]).plan(
                    self.planar(),goal,'FLIGHT','WALK',self.yaw)
            self.walk_goal=goal
        except ValueError as exc:
            self.warning_message='No hybrid route: '+str(exc)
            print('[HYBRID]',self.warning_message)
            return False
        self.state='TO_WALK';self.paused=False
        self.warning_message=''
        print('[HYBRID] Sensor-screened descent approved; mode-augmented path length',
              len(self.pending_hybrid_route))
        return True

    def request_flight(self):
        if self.state not in ('WALKING','WALK_HOLD','TO_WALK','LANDED'):
            return False
        self.pending_hybrid_route=None
        self.state='TO_FLIGHT';self.paused=False
        print('[HYBRID] Flight transition requested')
        return True

    def _replan_walk(self):
        if self.walk_goal is None:return False
        try:
            plan=HybridStatePlanner(self.make_planner()).plan(
                self.planar(),self.walk_goal,'WALK','WALK',self.yaw)
            self.walk_path=plan
            self.walk_index=1
            print('[HYBRID A*] walking states=',len(plan),'to',self.walk_goal)
            return True
        except ValueError as exc:
            self.warning_message=str(exc)
            self.paused=True
            print('[WALK NO PATH]',exc)
            return False

    def begin_walk(self):
        if self.pending_hybrid_route:
            path=[p for p in self.pending_hybrid_route if p.mode=='WALK']
            goal=self.walk_goal
            self.pending_hybrid_route=None
            self.walk_goal=goal
            self.walk_path=path
            self.walk_index=1
            self.state='WALKING'
            self.walk_phase=0.0
            self.walk_hold_elapsed=0.0
            print('[HYBRID A*] Unified FLIGHT->WALK plan activated;',len(path),'walk states')
            return
        try:
            goal,path=choose_safe_walk_goal(self.make_planner(),self.planar())
        except ValueError as exc:
            self.warning_message=str(exc)
            self.state='WALK_HOLD';self.walk_hold_elapsed=0.0
            print('[WALK BLOCKED]',exc)
            return
        self.walk_goal=goal
        self.walk_path=path
        self.walk_index=1
        self.state='WALKING'
        self.walk_phase=0.0
        self.walk_hold_elapsed=0.0
        print('[WALK] autonomous ground destination:',goal,'heading states:',len(path))

    def _set_pose(self,x,y,z):
        self.position=[x,y,z]
        self.position_field.setSFVec3f(self.position)
        self.rotation_field.setSFRotation([0,0,1,self.yaw])
        self.node.resetPhysics()

    def move_step(self):
        if self.paused:return
        if self.state=='TO_WALK':
            x,y,z=self.position
            step=max(0.65,0.6*self.speed)*self.dt
            z+=clamp(self.walk_height-z,-step,step)
            self._set_pose(x,y,z)
            if abs(z-self.walk_height)<=0.003:self.begin_walk()
        elif self.state=='WALKING':
            x,y,z=self.position
            if self.walk_index>=len(self.walk_path):
                self.state='WALK_HOLD';self.walk_hold_elapsed=0.0
                self.walk_photographed=False
                print('[WALK] target reached; inspecting area')
                return
            goal=self.walk_path[self.walk_index]
            dx,dy=goal.x-x,goal.y-y
            dist=math.hypot(dx,dy)
            if dist<0.11:
                self.walk_index+=1
            else:
                speed=self.walk_speed
                step=min(speed*self.dt,dist)
                x+=step*dx/dist;y+=step*dy/dist
                yaw=math.atan2(dy,dx)
                self.yaw+=clamp(angle_difference(yaw,self.yaw),-1.5*self.dt,1.5*self.dt)
                self.walk_phase+=2*math.pi*max(0.1,speed)*self.dt/0.85
            self._set_pose(x,y,self.walk_height)
        elif self.state=='WALK_HOLD':
            self.walk_hold_elapsed+=self.dt
            if not self.walk_photographed and self.walk_hold_elapsed>=0.75:
                self.walk_photographed=True
                self.capture('survey_camera', automatic=True, ground_inspection=True)
            if self.walk_hold_elapsed>=2.5:self.request_flight()
        elif self.state=='TO_FLIGHT':
            x,y,z=self.position
            z=min(self.altitude,z+max(0.7,0.65*self.speed)*self.dt)
            self._set_pose(x,y,z)
            if z>=self.altitude-0.003:
                self.state='FLYING'
                self.walk_goal=None
                self.replan(draw_overview=True)
                print('[HYBRID] Flight mode restored; survey mission resumed')
        else:
            super().move_step()

    def _update_gait(self):
        is_walk=self.state in ('WALKING','WALK_HOLD','TO_WALK')
        target=gait_angles(self.walk_phase) if self.state=='WALKING' else gait_angles(0.0)
        for leg in LEG_NAMES:
            hip=self.leg_fields.get(leg)
            knee=self.knee_fields.get(leg)
            if hip:hip.setSFRotation([0,1,0,target[leg] if is_walk else 0.0])
            if knee:
                bend=max(0.0,math.sin(self.walk_phase+(0 if leg in ('fl','rr') else math.pi)))
                knee.setSFRotation([0,1,0,(0.08+0.22*bend) if is_walk else 0.0])

    def update_rotors(self):
        super().update_rotors()
        self._update_gait()
        if self.state in ('TO_WALK','TO_FLIGHT'):
            for i,(field,direction,_) in enumerate(self.rotor_visuals):
                self.rotor_angle[i]=(self.rotor_angle[i]+self.rotor_speed*direction*self.dt)% (2*math.pi)
                field.setSFRotation([0,0,1,self.rotor_angle[i]])

    def foot_status(self):
        readings={}
        for leg,dev in self.walk_forces.items():
            try:
                v=dev.getValues()[:3]  # Webots returns a native pointer to three values.
                readings[leg]=[round(float(a),3) for a in v]
            except Exception:
                readings[leg]=None
        estimated=force_estimate(3.9,grounded=(self.state in ('WALKING','WALK_HOLD','ROLLING','ROLL_HOLD')))
        return {'raw_force_xyz_n':readings,
                'estimated_support_n':estimated,
                'note':'Raw Webots force-3d requires meaningful contact physics. '
                       'Support is an analytical weight estimate, not a sensor reading.'}

    def sense_obstacles(self):
        if not self.sensor_obstacle_enabled or self.paused:
            return
        if self.state not in ('FLYING','WALKING','ROLLING'):
            return
        t=self.robot.getTime()
        if t-self.last_sensor_check<0.30:return
        self.last_sensor_check=t
        device=self.sensor.get('ultrasonic_front')
        try:d=float(device.getValue()) if device else float('nan')
        except Exception:return
        detection=projected_obstacle(self.planar(),self.yaw,d,
                                    maximum=2.8 if self.state in ('WALKING','ROLLING') else 3.5)
        if detection is None:return
        if detection.distance_m<self.sensor_obstacle_stop_m:
            self.paused=True
            self.warning_message='EMERGENCY STOP: sensor obstacle at %.2fm'%d
            print('[PERCEPTION]',self.warning_message)
            return
        if t-self.sensor_obstacle_time<3.0:return
        grid=self.make_planner()
        cell=grid.xy_to_cell(detection.x,detection.y)
        if not grid.is_free(cell):
            return  # Existing mapped obstacle; do not add an identical unknown obstacle.
        new={'name':'Detected object (sensor)','x':detection.x,'y':detection.y,
             'sx':1.0,'sy':1.0,'height':1.2}
        if self.sensor_obstacle in self.obstacles:
            self.obstacles.remove(self.sensor_obstacle)
        self.obstacles.append(new)
        self.sensor_obstacle=new
        self.sensor_obstacle_time=t
        print('[PERCEPTION] Ultrasonic %.2fm -> dynamic obstacle and %s replan'
              %(d,self.state))
        self._replan_walk() if self.state in ('WALKING','ROLLING') else self.replan(draw_overview=True)

    def write_hybrid_status(self):
        out=self.captures_dir/self.session
        out.mkdir(parents=True,exist_ok=True)
        data={'sim_time_s':round(self.robot.getTime(),2),
              'mode':('WALK' if self.state in ('WALKING','WALK_HOLD','TO_WALK') else 'FLIGHT'),
              'state':self.state,
              'position_xyz_m':[round(v,3) for v in self.position],
              'walk_goal_xy':self.walk_goal,
              'walking_speed_mps':self.walk_speed,
              'sensor_obstacle':self.sensor_obstacle,
              'leg_hip_angles_rad':gait_angles(self.walk_phase),
              'feet':self.foot_status(),
              'warning':self.warning_message}
        tmp=out/'hybrid_status.tmp'
        tmp.write_text(json.dumps(data,indent=2),encoding='utf8')
        tmp.replace(out/'hybrid_status.json')

    def run(self):
        while self.robot.step(self.step_ms)!=-1:
            self.read_keys()
            self.sense_obstacles()
            self.update_rotors()
            self.move_step()
            now=self.robot.getTime()
            self.livo_stream.update(now,self.position,self.yaw)
            self.evaluate_terrain()
            if now-self.last_print>=float(self.settings.get('telemetry_every_s',2)):
                self.read_lidar()
                self.update_overlay()
                self.write_sensor_dashboard()
                self.write_hybrid_status()
                from console_telemetry import emit as emit_console_telemetry
                emit_console_telemetry(self, now)
                self.robot.setLabel(10,
                    'HYBRID %s | W walk | I fly | D sensor avoidance | %s' %
                    (self.state,self.warning_message[:65]),
                    .015,.36,.025,0x80FFCE,.02,'Arial')
                self.last_print=now
                print('[HYBRID] time %.1fs state %s position %s'%(now,self.state,
                      [round(v,2) for v in self.position]))

if __name__=='__main__':
    HybridMission().run()
