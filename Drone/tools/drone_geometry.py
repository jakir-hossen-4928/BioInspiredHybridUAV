"""Quadcopter geometry and camera views for the farm environment.

Run from the project root: python tools/create_worlds.py
This generator retains mapped farm geometry, routes and sensor devices.
No external PROTO is required.
"""
import math
from pathlib import Path
import survey_layout as survey

old = survey.old
ROOT = Path(__file__).resolve().parents[1]
ORIGINAL_MAVIC = old.mavic
SCALE = 2.30


def big_drone_visuals():
    """Local quadcopter geometry with visible propellers."""
    s = SCALE
    dark = old.col((.085, .105, .125), .49, .30)
    shell = old.col((.32, .38, .43), .48, .33)
    shell_light = old.col((.73, .77, .79), .41, .24)
    carbon = old.col((.040, .053, .065), .45, .35)
    graphite = old.col((.20, .24, .29), .45, .31)
    glass = old.col((.025, .055, .075), .09, .19)
    red = old.col((.97, .13, .08), .24, .08, emissive=(.2,.01,.01))
    green = old.col((.12, .91, .38), .24, .08, emissive=(.02,.24,.06))
    steel = old.col((.78, .82, .83), .31, .69)
    def pose(x,y,z,parts,rotation=None):
        return old.pose(x*s,y*s,z*s,parts,rotation)
    def box(x,y,z,material):
        return old.box(x*s,y*s,z*s,material)
    def sphere(r, material):
        return old.shape('Sphere { radius %.5f subdivision 3 }'%(r*s),material)
    def cyl(r,h,material):
        return old.shape('Cylinder { radius %.5f height %.5f }'%(r*s,h*s),material)
    parts = [
       'Transform { translation 0 0 %.5f scale %.5f %.5f %.5f children [ %s ] }' %
       (.055*s,.195*s,.113*s,.07*s,sphere(1/s,shell)),
       'Transform { translation %.5f 0 %.5f scale %.5f %.5f %.5f children [ %s ] }' %
       (.087*s,.038*s,.105*s,.097*s,.062*s,sphere(1/s,dark)),
       pose(-.035,0,.116,[box(.198,.164,.018,shell_light)]),
       pose(-.015,0,.127,[box(.12,.09,.006,carbon)]),
       pose(.13,0,.082,[box(.063,.175,.014,steel)]),
       pose(.013,0,-.021,[box(.23,.14,.046,graphite)]),
       pose(.147,0,-.077,[box(.060,.077,.050,carbon)]),
       pose(.181,0,-.077,[cyl(.028,.015,steel)],rotation=(0,0,1,math.pi/2)),
       pose(.195,0,-.077,[sphere(.022,glass)]),
       # The *real Webots camera device* is near the visually modelled gimbal lens.
       pose(.200,0,-.077,['Camera { name "camera" width 1920 height 1080 fieldOfView 1.05 near 0.03 far 170 }']),
       # Additional body panel details.
       pose(-.04,0,.137,[box(.11,.075,.008,shell_light)]),
       pose(-.149,0,.03,[box(.08,.16,.022,carbon)]),
       pose(.055,.112,.025,[box(.115,.011,.015,dark)]),
       pose(.055,-.112,.025,[box(.115,.011,.015,dark)])
    ]
    for x in (-.12,.12):
        for y in (-.095,.095):
            parts.append(pose(x,y,-.065,[box(.018,.022,.112,carbon)]))
            parts.append(pose(x,y,-.119,[box(.077,.024,.013,dark)]))
    rotors=[('front left propeller',.270,.222,1),
            ('front right propeller',.270,-.222,-1),
            ('rear left propeller',-.270,.222,-1),
            ('rear right propeller',-.270,-.222,1)]
    for name,x,y,sign in rotors:
        a = math.atan2(y,x)
        arm_len = math.hypot(x,y)
        parts.append(pose(x*.50,y*.50,.057,[box(arm_len+.015,.038,.035,carbon)],rotation=(0,0,1,a)))
        parts.append(pose(x,y,.091,[cyl(.047,.052,graphite)],rotation=(1,0,0,math.pi/2)))
        parts.append(pose(x,y,.124,[sphere(.018,green if x>0 else red)]))
        # Four genuinely motor-driven blades, only for visual animation, not thrust physics.
        anchor = (x*s,y*s,.147*s)
        rotor=('HingeJoint {\n'
               ' jointParameters HingeJointParameters { axis 0 0 1 anchor %.5f %.5f %.5f }\n'
               ' device [ RotationalMotor { name "%s" maxVelocity 400 maxTorque 1.2 } ]\n'
               ' endPoint Solid { translation %.5f %.5f %.5f name "%s visual rotor"\n'
               ' children [\n'
               '  Pose { translation 0 0 %.5f children [ Shape { appearance %s geometry Box { size %.5f %.5f %.5f } } ] }\n'
               '  Pose { translation 0 0 %.5f rotation 0 0 1 1.5707963 children [ Shape { appearance %s geometry Box { size %.5f %.5f %.5f } } ] }\n'
               '  Shape { appearance %s geometry Cylinder { radius %.5f height %.5f } }\n'
               ' ] boundingObject Sphere { radius %.5f }\n'
               ' physics Physics { density -1 mass 0.023 }\n'
               ' }\n}' %
               (*anchor,name,*anchor,name,
                .007*s,carbon,.390*s,.022*s,.005*s,
                .007*s,carbon,.390*s,.022*s,.005*s,
                steel,.027*s,.015*s,.02*s))
        parts.append(rotor)
    return parts


def bigger_mavic(x,y):
    txt=ORIGINAL_MAVIC(x,y)
    # Old mavic helper uses the globally overridden big_drone_visuals().
    replacements = {
      'translation %g %g .16' % (x,y): 'translation %g %g .39' % (x,y),
      'translation 0 0 0.16': 'translation 0 0 0.370',
      'translation .19 0 .015': 'translation .440 0 .034',
      'translation 0 0 -.067': 'translation 0 0 -.154',
      'size .35 .22 .10': 'size .81 .51 .24',
      'Physics { mass 1.15 density -1 }': 'Physics { mass 3.9 density -1 }',
      'size .026 .025 .025': 'size .060 .058 .058',
    }
    for before,after in replacements.items():
        assert before in txt,before
        txt=txt.replace(before,after)
    # Foot devices are located under the 2.3x larger landing skids.
    for side,dx,dy in [('fl',.08,.07),('fr',.08,-.07),('rl',-.10,.07),('rr',-.10,-.07)]:
        before = 'translation %.3f %.3f -.095 name "foot_%s"' % (dx,dy,side)
        after = 'translation %.4f %.4f -.2185 name "foot_%s"' % (dx*SCALE,dy*SCALE,side)
        assert before in txt,before
        txt=txt.replace(before,after)
    return txt


def camera(mode):
    if mode=='follow':
        # Dedicated *close* following shot shows the full-size drone clearly.
        pos=(-52.4,-51.2,3.9); target=(-49,-47,.82)
        extra='follow "Drone" followType "Tracking Shot" followSmoothness 0.18'
        fov=.70
    elif mode=='closeup':
        pos=(-50.8,-49.7,2.5); target=(-49,-47,.82)
        extra='follow "Drone" followType "Tracking Shot" followSmoothness 0.08'
        fov=.74
    elif mode=='top':
        pos=(0,-1,150);target=(0,0,0)
        extra='';fov=.94
    else:
        pos=(90,-97,98);target=(0,0,1.8)
        extra='';fov=.94
    return ('Viewpoint { orientation %s position %s near .15 far 680 '
            'fieldOfView %.4f ambientOcclusionRadius 1.3 %s }' %
            (old.vec(survey.cinematic.correct_look_at(pos,target)),old.vec(pos),fov,extra))


def generate_world(mode,photo_size=(1920,1080)):
    txt = survey.create_world(mode)
    # The downward camera already faces -Z; upgrade size and positioning.
    cam_old=('Camera { name "survey_camera" translation 0.04 0 -0.17 '
             'rotation 0 1 0 1.57079632679 width 800 height 600 '
             'fieldOfView 1.16 near 0.08 far 130 }')
    cam_new=('Camera { name "survey_camera" translation 0.09 0 -0.20 '
             'rotation 0 1 0 1.57079632679 width %d height %d '
             'fieldOfView 1.12 near 0.06 far 175 }' % photo_size)
    assert txt.count(cam_old)==1
    txt=txt.replace(cam_old,cam_new,1)
    # Front camera dimensions are generated in big_drone_visuals.
    front='name "camera" width 1920 height 1080 fieldOfView 1.05'
    assert txt.count(front)==1
    if photo_size!=(1920,1080):
        txt=txt.replace(front,'name "camera" width %d height %d fieldOfView 1.05'%photo_size,1)
        assert txt.count('DEF DRONE Robot')==1
    assert txt.count('HingeJoint {')==4
    return txt


def main():
    old.drone_visuals=big_drone_visuals
    old.mavic=bigger_mavic
    old.camera_block=camera
    out=ROOT/'worlds'
    out.mkdir(exist_ok=True)
    for p in out.glob('*.wbt'):p.unlink()
    profiles={
        'farm.wbt':('follow',(1920,1080)),
        'overview.wbt':('overview',(1920,1080)),
        'follow.wbt':('follow',(1920,1080)),
        'closeup.wbt':('closeup',(1920,1080)),
        'top.wbt':('top',(1920,1080)),
        'photos.wbt':('follow',(3840,2160)),
    }
    for file,(mode,size) in profiles.items():
        contents=generate_world(mode,size)
        (out/file).write_text(contents,encoding='utf-8')
        print('Created',file, 'camera=%dx%d'%size,
              'bytes=',len(contents.encode('utf-8')))
    print('Drone visual size factor=',SCALE,'Photo stops=',len(old.S['photo_stops']))

if __name__=='__main__':
    main()
