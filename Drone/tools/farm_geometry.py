"""Webots R2025a farm world generator. Pure Python, no runtime dependencies.
Visual farm objects & local original textures; only the official DJI Mavic model uses webots://.
ENU, Z up. Flight is demonstrative kinematic A*, not PX4/SLAM/FAST-LIVO2.
"""
import json, math, random
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
S = json.loads((ROOT / 'controllers'/'drone'/'scene.json').read_text())
RNG = random.Random(9234)

def vec(vals): return ' '.join(f'{float(a):.5g}' for a in vals)
def col(rgb, rough=.86, metal=0, transp=0, emissive=None):
    s=f'PBRAppearance {{ baseColor {vec(rgb)} roughness {rough:g} metalness {metal:g}'
    if transp: s+=f' transparency {transp:g}'
    if emissive: s+=f' emissiveColor {vec(emissive)} emissiveIntensity 1'
    return s+' }'

def tex(path, repeat=1, rough=.87):
    return ('PBRAppearance { baseColorMap ImageTexture { url [ "../textures/%s" ] } '
            'textureTransform TextureTransform { scale %g %g } roughness %g metalness 0 }'
            % (path, repeat, repeat, rough))

MAT = {
    'grass':tex('grass.png',18,1),
    'soil':tex('rich_soil.png',2),
    'dirt':tex('dry_dirt.png',3),
    'gravel':tex('gravel.png',4),
    'wood':tex('farm_wood.png',3),
    'barn':tex('barn_red_wood.png',2),
    'metal_roof':tex('rustic_roof.png',2),
    'brick':tex('farm_bricks.png',2),
    'hay':tex('hay_bales.png',2),
    'water':tex('water.png',3,.1),
    'fence':col((.53,.37,.19)),
    'darkwood':col((.24,.15,.1)),
    'edge':col((.14,.20,.13)),
    'stone':col((.60,.61,.57)),
    'metal':col((.46,.51,.52),.38,.75),
    'steel':col((.28,.33,.37),.47,.68),
    'glass':col((.48,.83,.91),.12,.12,.56),
    'window':col((.18,.39,.48),.14,.09,.23),
    'roof_red':col((.52,.19,.13),.70),
    'leaf':col((.21,.42,.19),.93),
    'leaf2':col((.32,.56,.18),.94),
    'leaf3':col((.17,.34,.16),.98),
    'orange':col((.99,.47,.075),.74),
    'red':col((.78,.17,.11),.69),
    'tractor':col((.78,.12,.08),.63),
    'wheel':col((.09,.095,.085),.95),
    'rim':col((.86,.75,.51),.56,.52),
    'white':col((.93,.89,.78)),
    'green':col((.11,.55,.30),.71),
    'yellow':col((.98,.73,.14),.63),
    'corn':col((.24,.58,.13),.87),
    'wheat':col((.72,.60,.16),.96),
    'vegetables':col((.16,.56,.18),.99),
    'soil_row':col((.37,.25,.15),1),
    'lining':col((.9,.86,.68),.9),
    'water_blue':col((.12,.50,.76),.16,.2,.11),
}

def shape(geometry, material): return f'Shape {{ appearance {MAT.get(material,material)} geometry {geometry} }}'
def boxgeom(x,y,z): return f'Box {{ size {x:.4f} {y:.4f} {z:.4f} }}'
def box(x,y,z, material): return shape(boxgeom(x,y,z),material)
def pose(x,y,z,parts,rotation=None):
    r='' if rotation is None else ' rotation '+vec(rotation)
    return f'Pose {{ translation {x:.4f} {y:.4f} {z:.4f}{r} children [\n'+'\n'.join(parts)+'\n] }'
def cube(x,y,z,sx,sy,sz,material,rotation=None): return pose(x,y,z,[box(sx,sy,sz,material)],rotation)
def cyl(x,y,z,r,h,material,rotation=None): return pose(x,y,z,[shape(f'Cylinder {{ radius {r:.4f} height {h:.4f} }}',material)],rotation)
def cone(x,y,z,r,h,material): return pose(x,y,z,[shape(f'Cone {{ bottomRadius {r:.4f} height {h:.4f} }}',material)])
def sphere(x,y,z,r,material): return pose(x,y,z,[shape(f'Sphere {{ radius {r:.4f} subdivision 2 }}',material)])
def solid(ob,parts):
    sx,sy,h=ob['sx'],ob['sy'],ob['height']
    return (f'DEF FARM_{ob["kind"].upper()} Solid {{\n'
            f' translation {ob["x"]:.3f} {ob["y"]:.3f} 0\n'
            f' name "{ob["name"]}"\n'
            ' children [\n'+'\n'.join(parts)+'\n ]\n'
            f' boundingObject Pose {{ translation 0 0 {h/2:.4f} children [ {boxgeom(sx,sy,h)} ] }}\n'
            '}')

def pitched_roof(sx,sy,eave,rise,material,overhang=.30):
    width=sx/2+overhang
    length=math.hypot(width,rise)
    a=math.atan2(rise,width)
    return [
        cube(-width/2,0,eave+rise/2,length,sy+2*overhang,.14,material,(0,1,0,-a)),
        cube(width/2,0,eave+rise/2,length,sy+2*overhang,.14,material,(0,1,0,a)),
        # Ridge timber
        cube(0,0,eave+rise,.18,sy+2*overhang,.18,'metal_roof'),
    ]

def barn(ob):
    sx,sy=ob['sx'],ob['sy']
    eave=4.40
    parts=[cube(0,0,eave/2,sx,sy,eave,'barn'),
           cube(0,-sy/2-.045,1.67,3.50,.09,3.25,'darkwood'),
           cube(0,-sy/2-.085,3.8,1.05,.10,.64,'white')]
    for x in [-1.05,0,1.05]:
        parts.append(cube(x,-sy/2-.115,1.67,.07,.12,3.1,'metal'))
    for j in range(4):
        parts.append(cube(0,-sy/2-.125,.41+j*.75,3.5,.035,.04,'wood'))
    parts += pitched_roof(sx,sy,eave,1.65,'metal_roof')
    for x in (-sx/2+.85,sx/2-.85):
        parts.append(cube(x,-sy/2-.05,3.70,.58,.09,.42,'window'))
    # Barn loft ventilation and side wood framing.
    for y in [-sy/2+.3,sy/2-.3]:
        parts.append(cube(-sx/2-.07,y,2.1,.11,.12,3.9,'wood'))
    return solid(ob,parts)

def farmhouse(ob):
    sx,sy=ob['sx'],ob['sy']
    wall_h=4.35
    parts=[cube(0,0,wall_h/2,sx,sy,wall_h,'brick'),
           cube(0,-sy/2-.06,1.0,1.05,.10,1.96,'wood'),
           cube(0,-sy/2-.16,2.05,1.6,.30,.13,'stone')]
    for side in (-1,1):
        for z in (1.55,3.30):
            parts.append(cube(side*1.8,-sy/2-.048,z,.82,.10,.90,'window'))
            parts.append(cube(side*1.8,-sy/2-.11,z-.48,.95,.16,.09,'white'))
        for z in (1.55,3.30):
            parts.append(cube(side*(sx/2+.04),0,z,.075,1.0,.9,'window'))
    parts += pitched_roof(sx,sy,wall_h,1.22,'roof_red')
    parts += [cube(sx*.22,sy*.12,5.05,.48,.48,1.14,'brick'),
              cube(sx*.22,sy*.12,5.64,.62,.62,.13,'stone')]
    # White porch columns and porch canopy.
    parts.append(cube(0,-sy/2-1.0,2.30,3.50,1.90,.15,'roof_red'))
    for x in (-1.5,1.5):parts.append(cube(x,-sy/2-1.83,1.16,.13,.13,2.31,'white'))
    return solid(ob,parts)

def greenhouse(ob):
    sx,sy=ob['sx'],ob['sy']; h=3.1
    parts=[cube(0,0,.17,sx+.25,sy+.25,.34,'stone'),
           cube(0,0,1.75,sx,sy,3.1,'glass'),
           cube(0,-sy/2-.05,1.3,1.4,.15,2.3,'window')]
    parts+=pitched_roof(sx,sy,h,1.0,'glass',.08)
    for x in [-sx/2,-sx/4,0,sx/4,sx/2]:
        parts.append(cube(x,-sy/2-.055,1.6,.075,.11,3.12,'metal'))
        parts.append(cube(x,sy/2+.055,1.6,.075,.11,3.12,'metal'))
    for y in [-sy/2,0,sy/2]:
        for x in (-sx/2,sx/2):
            parts.append(cube(x,y,1.6,.11,.09,3.1,'metal'))
    parts.append(cube(0,0,3.12,sx,sy,.075,'metal'))
    return solid(ob,parts)

def silo(ob):
    r=1.7; h=7.0
    parts=[cyl(0,0,h/2,r,h,'metal_roof'),cone(0,0,7.65,1.9,1.7,'metal'),
           cyl(0,0,1.3,1.74,.08,'steel')]
    for z in [1.3,2.6,3.9,5.2,6.5]:
        parts.append(cyl(0,0,z,1.725,.055,'steel'))
    for z in [.7,1.05,1.4,1.75,2.1,2.45,2.8,3.15,3.5,3.85,4.2,4.55,4.9,5.25,5.6,5.95,6.3]:
        parts.append(cube(0,-1.77,z,.46,.07,.045,'steel'))
    parts.append(cube(0,-1.75,3.8,.07,.07,6.5,'steel'))
    return solid(ob,parts)

def tower(ob):
    parts=[]
    for x in (-1.15,1.15):
        for y in (-1.15,1.15):
            parts.append(cube(x,y,2.65,.15,.15,5.3,'steel'))
    for z in (1.35,3.2,5.1):
        for x in (-1.15,1.15): parts.append(cube(x,0,z,.08,2.45,.08,'steel'))
        for y in (-1.15,1.15): parts.append(cube(0,y,z,2.45,.08,.08,'steel'))
    parts += [cyl(0,0,5.4,1.55,1.92,'water_blue'),
              cyl(0,0,6.4,1.62,.16,'steel'),
              cube(0,-1.60,5.65,1.24,.07,.56,'white')]
    return solid(ob,parts)

def shed(ob):
    sx,sy=ob['sx'],ob['sy']
    parts=[cube(0,0,1.55,sx,sy,3.1,'wood'),
           cube(0,-sy/2-.085,1.45,sx*.77,.10,2.5,'darkwood'),
           cube(0,-sy/2-.15,1.6,sx*.55,.04,.13,'steel')]
    parts += pitched_roof(sx,sy,3.12,1.20,'metal_roof')
    for x in (-sx/2+.4,sx/2-.4):
        parts.append(cube(x,-sy/2-.11,1.5,.12,.10,2.9,'wood'))
    return solid(ob,parts)

def haystore(ob):
    sx,sy=ob['sx'],ob['sy'];parts=[]
    for ix in range(3):
        for iy in range(2):
            for iz in range(2):
                bx=-sx/2+.78+ix*1.42
                by=-sy/2+.90+iy*1.52
                parts.append(cube(bx,by,.52+iz*1.01,1.37,1.40,.96,'hay'))
                parts.append(cube(bx,by,.52+iz*1.01,.025,1.42,.99,'darkwood'))
    return solid(ob,parts)

def tractor(ob):
    pieces=[cube(0,0,.78,2.5,1.40,1.1,'tractor'),
            cube(.7,0,1.52,1.1,1.25,.85,'glass'),
            cube(.7,0,1.97,1.4,1.58,.12,'tractor'),
            cube(-1.0,0,1.36,.15,.15,1.25,'steel'),
            cube(-1.27,0,.72,.12,.72,.45,'steel'),
            cube(1.28,0,.70,.08,1.30,.22,'metal')]
    for x,rad in [(-.78,.59),(.89,.39)]:
        for y in (-.77,.77):
            # Rotate cylinders so their axles lie along Y.
            pieces.append(cyl(x,y,rad,rad,.30,'wheel',(1,0,0,math.pi/2)))
            pieces.append(cyl(x,y+(-.166 if y<0 else .166),rad,rad*.49,.035,'rim',(1,0,0,math.pi/2)))
    return solid(ob,pieces)

def fruit_tree(x,y,s,id):
    h=2.9*s
    parts=[cyl(0,0,1.23*s,.20*s,2.46*s,'darkwood'),
           sphere(0,0,2.8*s,1.16*s,'leaf'),
           sphere(-.49*s,.09*s,3.24*s,.8*s,'leaf2'),
           sphere(.48*s,-.25*s,3.29*s,.77*s,'leaf3')]
    rng=random.Random(id+220)
    for _ in range(11):
        ang=rng.random()*math.tau
        r=(.49+rng.random()*.49)*s
        z=(2.7+rng.random()*.85)*s
        parts.append(sphere(r*math.cos(ang),r*math.sin(ang),z,.10*s,'orange'))
    return (f'Solid {{ translation {x:.3f} {y:.3f} 0 name "Orange Tree {id}" '
            'children [ '+' '.join(parts)+' ] '
            f'boundingObject Pose {{ translation 0 0 {h:.3f} children [ Sphere {{ radius {1.13*s:.3f} }} ] }} '+'}')

def orchard(ob):
    parts=[cube(0,0,.043,ob['sx'],ob['sy'],.07,'grass')]
    # Tree locations relative to orchard center, well within planner safety zone.
    for row in range(3):
        for ix in range(3):
            x=ob['x']+(-2.35+ix*2.35)
            y=ob['y']+(-2.4+row*2.35)
            parts.append(fruit_tree(x,y,.78+(ix+row)%3*.04,ix+3*row))
    return '\n'.join(parts)

def field_mesh(spec):
    """Real 3D rows of lightweight procedural leafy triangles, packed as one IndexedFaceSet."""
    rng=random.Random(422+len(spec['name']))
    kind=spec['kind'];sx,sy=spec['sx'],spec['sy'];rows=int(spec['rows']);
    parts=[cube(0,0,.03,sx,sy,.085,'soil')]
    vertices=[];faces=[]; wheat_heads=[]; head_faces=[]
    def tri(group,indices,a,b,c):
        i=len(group);group.extend([a,b,c]);indices.append((i,i+1,i+2))
    count=0
    for row in range(rows):
        x=-sx/2+.4+(row/(rows-1))*(sx-.8)
        parts.append(cube(x,0,.095,.18,sy-.55,.085,'soil_row'))
        n=max(9,int((sy-.6)/(.44 if kind=='corn' else .38)))
        for j in range(n):
            y=-sy/2+.3+j*(sy-.6)/max(1,n-1)+rng.uniform(-.11,.11)
            px=x+rng.uniform(-.078,.078)
            height={'corn':1.22,'wheat':.66,'lettuce':.27}[kind]*(.76+rng.random()*.30)
            width={'corn':.23,'wheat':.07,'lettuce':.18}[kind]
            # Each plant is a crossed strip of slanted leaves; one batched mesh per crop.
            for ang in (0, math.pi/2, math.pi, 3*math.pi/2):
                ang+=rng.uniform(-.14,.14)
                dx=math.cos(ang);dy=math.sin(ang)
                low=.10
                high=height+low
                tip=(px+dx*width*1.25,y+dy*width*1.25,high*.79)
                side=(px-dy*width*.55,y+dx*width*.55,high*.45)
                tri(vertices,faces,(px, y, low),side,tip)
                tri(vertices,faces,(px, y, low),tip,(px+dy*width*.55,y-dx*width*.55,high*.45))
            if kind=='wheat':
                hh=.09+height
                tri(wheat_heads,head_faces,(px-.035,y,hh-.20),(px+.035,y,hh-.20),(px,y,hh+.13))
            count+=1
    coords=' '.join('%g %g %g' % tuple(round(v,4) for v in p) for p in vertices)
    indexes=' '.join(f'{a} {b} {c} -1' for a,b,c in faces)
    parts.append(f'Shape {{ appearance {MAT[kind if kind in MAT else "vegetables"]} '
                 'geometry IndexedFaceSet { coord Coordinate { point [ '
                 +coords+' ] } coordIndex [ '+indexes+' ] } }')
    if wheat_heads:
        coords=' '.join('%g %g %g' % tuple(round(v,4) for v in p) for p in wheat_heads)
        indexes=' '.join(f'{a} {b} {c} -1' for a,b,c in head_faces)
        parts.append(f'Shape {{ appearance {MAT["yellow"]} geometry IndexedFaceSet {{ '
                     'coord Coordinate { point [ '+coords+' ] } coordIndex [ '+indexes+' ] } }')
    return pose(spec['x'],spec['y'],0,parts),count

def fence_around():
    parts=[]
    # Fence below planner outer boundary; open gap at south-west entrance.
    for y in range(-28,29,3):
        for x in (-28,28):
            if (x==-28 and y<-21): continue
            parts.append(cube(x,y,.77,.16,.16,1.53,'wood'))
    for x in range(-28,29,3):
        for y in (-28,28):
            if y==-28 and x<-20: continue
            parts.append(cube(x,y,.77,.16,.16,1.53,'wood'))
    for x in (-28,28):
        for z in (.60,1.12):
            parts.append(cube(x,3.0 if x==-28 else 0,z,.10,47 if x==-28 else 56,.10,'wood'))
    for y in (-28,28):
        for z in (.60,1.12):parts.append(cube(3.5 if y==-28 else 0,y,z,49 if y==-28 else 56,.09,.10,'wood'))
    return 'Group { children [ '+'\n'.join(parts)+' ] }'

def dirt_road(a,b,width=2.1):
    ax,ay=a;bx,by=b
    length=math.hypot(bx-ax,by-ay)
    angle=math.atan2(by-ay,bx-ax)
    return cube((ax+bx)/2,(ay+by)/2,.024,length,width,.052,'dirt',(0,0,1,angle))

def round_bale(x,y,angle=0):
    return pose(x,y,.53,[shape('Cylinder { radius .55 height 1.0 }', 'hay')],(0,0,1,angle))

def irrigation_pipes():
    parts=[]
    for xx in (2.1,6.1,10.1,14.1):
        parts.append(cube(xx,-15.0,.12,.055,11.2,.075,'steel'))
        for yy in (-18.9,-15.0,-11.1):
            parts.append(cyl(xx,yy,.37,.045,.50,'metal'))
            parts.append(sphere(xx,yy,.65,.11,'water_blue'))
    return '\n'.join(parts)

def cows():
    out=[]
    for i,(x,y) in enumerate([(-3,23),(-.2,24.4)]):
        p=[sphere(0,0,1.02,.71,'white'),sphere(.72,0,1.18,.40,'white'),
           sphere(.98,-.05,1.03,.19,'brick'),
           sphere(-.25,.35,1.32,.23,'wheel'),sphere(.20,-.43,1.22,.27,'wheel')]
        for px in (-.43,.43):
            for py in (-.33,.33):p.append(cube(px,py,.40,.16,.17,.72,'white'))
        p.append(cube(-.82,0,1.05,.59,.06,.06,'white'))
        out.append(pose(x,y,0,p, (0,0,1,.12*i)))
    return '\n'.join(out)

def landing_pad(x,y,main=True):
    surface='green' if main else 'yellow'
    return pose(x,y,.035,[cyl(0,0,.02,1.02,.062,'stone'),
                         cyl(0,0,.069,.78,.030,surface),
                         cube(0,0,.090,.12,.84,.012,'white'),
                         cube(0,0,.091,.68,.12,.012,'white')])

def hay_obstacle():
    # DEF dynamic Solid at z=-10 initially; its collider moves to flight route upon pressing O.
    return ('DEF DYNAMIC_OBSTACLE Solid { translation 0 0 -10 name "Temporary Hay Bale Barrier" '
            'children [ '+cube(0,0,0,1.55,1.55,2.70,'hay')+' ] '
            'boundingObject Box { size 1.55 1.55 2.70 } }')

def look_at(pos,target):
    fx,fy,fz=[target[i]-pos[i] for i in range(3)]
    n=math.sqrt(fx*fx+fy*fy+fz*fz)
    f=(fx/n,fy/n,fz/n)
    left=(-f[1],f[0],0)
    sl=math.hypot(left[0],left[1]); left=tuple(v/sl for v in left)
    up=(f[1]*left[2]-f[2]*left[1], f[2]*left[0]-f[0]*left[2], f[0]*left[1]-f[1]*left[0])
    m=((f[0],left[0],up[0]),(f[1],left[1],up[1]),(f[2],left[2],up[2]))
    a=math.acos(max(-1,min(1,(m[0][0]+m[1][1]+m[2][2]-1)/2)))
    if a<1e-8: return (0,0,1,0)
    d=2*math.sin(a)
    if abs(d)<1e-6: return (0,0,1,a)
    return ((m[2][1]-m[1][2])/d,(m[0][2]-m[2][0])/d,(m[1][0]-m[0][1])/d,a)

def camera_block(mode):
    if mode=='follow':
        pos=(-33,-35,12); target=(S['start'][0],S['start'][1],.8)
        extra='follow "Autonomous DJI Mavic 2 PRO" followType "Tracking Shot" followSmoothness 0.19'
        fov=.83
    elif mode=='top':
        pos=(2,-11,72);target=(0,0,0);extra='';fov=.90
    else:
        pos=(38,-50,44);target=(0,0,1.2);extra='';fov=.91
    return (f'Viewpoint {{ orientation {vec(look_at(pos,target))} position {vec(pos)} '
            f'near .20 far 240 fieldOfView {fov} ambientOcclusionRadius 1.9 {extra} }}')

def drone_visuals():
    """Standalone Mavic-style quadcopter. Native Webots nodes only, no EXTERNPROTO.

    It is a detailed educational replica, NOT an official DJI mesh. Kinematic
    navigation (Supervisor) remains as in previous version.
    """
    dark = col((.13,.16,.19),.58,.22)
    shell = col((.36,.40,.44),.52,.32)
    shell_light = col((.68,.72,.75),.5,.24)
    carbon = col((.07,.085,.105),.54,.36)
    graphite = col((.21,.25,.28),.54,.31)
    glass = col((.035,.065,.075),.11,.16)
    red = col((.94,.13,.08),.4,.14,emissive=(.2,.015,.005))
    green = col((.09,.87,.3),.4,.11,emissive=(.01,.25,.04))
    steel = col((.75,.77,.76),.36,.7)
    parts = [
        # Central aerodynamic shell / dorsal battery cover
        'Transform { translation 0 0 .055 scale .195 .113 .070 children [ '+shape('Sphere { radius 1 subdivision 3 }',shell)+' ] }',
        'Transform { translation .087 0 .038 scale .105 .097 .062 children [ '+shape('Sphere { radius 1 subdivision 3 }',dark)+' ] }',
        pose(-.035,0,.116,[box(.198,.164,.018,shell_light)]),
        pose(-.015,0,.127,[box(.12,.09,.006,carbon)]),
        pose(.13,0,.082,[box(.063,.175,.014,steel)]),
        # Lower electronics housing and camera gimbal
        pose(.013,0,-.021,[box(.23,.14,.046,graphite)]),
        pose(.147,0,-.077,[box(.060,.077,.050,carbon)]),
        pose(.181,0,-.077,[shape('Cylinder { radius .028 height .015 }',steel)],rotation=(0,0,1,math.pi/2)),
        pose(.195,0,-.077,[shape('Sphere { radius .022 subdivision 3 }',glass)]),
        # RGB camera is a functional device, facing +X (Webots Camera points along +X).
        pose(.193,0,-.077,['Camera { name "camera" width 640 height 360 near .02 far 90 }']),
        # Four lower landing struts
    ]
    for x in (-.12,.12):
        for y in (-.095,.095):
            parts.append(pose(x,y,-.065,[box(.018,.022,.112,carbon)]))
            parts.append(pose(x,y,-.119,[box(.077,.024,.013,dark)]))
    # Four diagonal carbon arms and motor caps / navigation lights
    rotors=[('front left propeller',.270,.222,1),
            ('front right propeller',.270,-.222,-1),
            ('rear left propeller',-.270,.222,-1),
            ('rear right propeller',-.270,-.222,1)]
    for name,x,y,sign in rotors:
        a=math.atan2(y,x)
        arm_len=math.hypot(x,y)
        parts.append(pose(x*.50,y*.50,.057,[box(arm_len+.015,.038,.035,carbon)],rotation=(0,0,1,a)))
        parts.append(pose(x,y,.091,[shape('Cylinder { radius .047 height .052 }',graphite)],rotation=(1,0,0,math.pi/2)))
        light=(green if x>0 else red)
        parts.append(pose(x,y,.124,[shape('Sphere { radius .018 subdivision 2 }',light)]))
        # Motorized rotor blades use real Webots HingeJoint/RotationalMotor nodes.
        # This is for propeller visualization only, not thrust dynamics.
        rotor = ('HingeJoint {\n'
                 ' jointParameters HingeJointParameters { axis 0 0 1 anchor %.4f %.4f .147 }\n'
                 ' device [ RotationalMotor { name "%s" maxVelocity 400 maxTorque 1.2 } ]\n'
                 ' endPoint Solid { translation %.4f %.4f .147 name "%s visual rotor"\n'
                 ' children [\n'
                 '   Pose { translation 0 0 .007 children [ Shape { appearance %s geometry Box { size .390 .022 .005 } } ] }\n'
                 '   Pose { rotation 0 0 1 1.5708 translation 0 0 .007 children [ Shape { appearance %s geometry Box { size .390 .022 .005 } } ] }\n'
                 '   Shape { appearance %s geometry Cylinder { radius .027 height .015 } }\n'
                 ' ]\n'
                 ' boundingObject Sphere { radius .020 }\n'
                 ' physics Physics { density -1 mass 0.015 }\n'
                 ' }\n}'
                 % (x,y,name,x,y,name,carbon,carbon,steel))
        parts.append(rotor)
    return parts


def mavic(x,y):
    """Legacy function name retained to keep generator hooks compatible."""
    body = """
    Altimeter { name "altimeter" }
    Accelerometer { name "accelerometer" }
    Lidar {
      translation 0 0 0.16
      name "lidar"
      horizontalResolution 240
      numberOfLayers 12
      fieldOfView 6.28318
      verticalFieldOfView 0.68
      minRange 0.12
      maxRange 18
      type "fixed"
      projection "cylindrical"
    }
    DistanceSensor {
      translation .19 0 .015
      name "ultrasonic_front"
      type "sonar"
      numberOfRays 7
      aperture .40
      lookupTable [ 0 0 0, 8 8 0 ]
    }
    DistanceSensor {
      translation 0 0 -.067
      rotation 0 1 0 1.570796327
      name "ultrasonic_down"
      type "sonar"
      numberOfRays 5
      aperture .28
      lookupTable [ 0 0 0, 8 8 0 ]
    }
    Compass { name "compass" }
    GPS { name "gps" }
    Gyro { name "gyro" }
    InertialUnit { name "inertial unit" }
    LED { name "front left led" }
    LED { name "front right led" }
"""
    for side,dx,dy in [('fl',.08,.07),('fr',.08,-.07),('rl',-.10,.07),('rr',-.10,-.07)]:
        body+=('TouchSensor { translation %.3f %.3f -.095 name "foot_%s" '
               'type "bumper" children [ %s ] boundingObject Box { size .026 .025 .025 } }\n'
               % (dx,dy,side,box(.026,.025,.025,'steel')))
    parts=drone_visuals()
    return (f'DEF DRONE Robot {{ translation {x:g} {y:g} .16 '
            'rotation 0 0 1 0 name "Drone" '
            'supervisor TRUE controller "drone" children [\n'+body+'\n'
            + '\n'.join(parts) +'\n] '
            'boundingObject Box { size .35 .22 .10 } '
            'physics Physics { mass 1.15 density -1 } }')

def scene(mode):
    out=['#VRML_SIM R2025a utf8',
         'WorldInfo { title "Farm Autonomous Navigation - Local Farm Quadcopter" '
         'info [ "Realistic farm visuals + known-map A*. Kinematic flight, NOT FAST-LIVO2 / Hybrid A* or real control physics." ] '
         'basicTimeStep 32 coordinateSystem "ENU" gravity 0 }',
         camera_block(mode),
         'Background { skyColor [ 0.56 0.73 0.90 ] }',
         'DirectionalLight { direction -0.55 -0.45 -1 intensity 2.2 ambientIntensity .58 castShadows TRUE }',
         'Solid { translation 0 0 -0.16 name "Farm terrain" children [ '+box(64,64,.32,'grass')+' ] '
         'boundingObject Box { size 64 64 .32 } }',
         fence_around(),
         # Farm roads and tracks
         dirt_road((-28,-23),(-20,-23),2.3),
         dirt_road((-23,-26),(-23,26),1.85),
         dirt_road((-23,-.8),(23,-.8),2.05),
         dirt_road((-23,-23),(25,-23),2.20),
         dirt_road((23,-23),(23,-4),1.70),
         dirt_road((-1,-23),(-1,-17),1.35),
         dirt_road((6,2),(6,9),1.30),
         ]
    for f in S['fields']:
        mesh,n=field_mesh(f)
        out.append('# '+f['name']+' - '+str(n)+' individual 3D crop plants')
        out.append(mesh)
    out.append('# Farm irrigation risers and water piping')
    out.append(irrigation_pipes())
    for ob in S['obstacles']:
        if ob['kind']=='orchard':out.append(orchard(ob))
        else:out.append({'barn':barn,'farmhouse':farmhouse,'greenhouse':greenhouse,
                         'silo':silo,'water':tower,'shed':shed,'hay':haystore,
                         'tractor':tractor}[ob['kind']](ob))
    # Scenic ponds, pumping station and bale stacks.
    out.append(cyl(22,-16,.055,2.40,.055,'water_blue'))
    out.append(cyl(22,-16,.008,2.68,.06,'stone'))
    for a in range(18):
        theta=a*2*math.pi/18
        out.append(sphere(22+2.65*math.cos(theta),-16+2.65*math.sin(theta),.15,.32,'stone'))
    for x,y in [(20.3,-17.4),(23.4,-14.8),(20.1,-15)]:
        out.append(cyl(x,y,.55,.045,1.1,'leaf3'))
    for x,y in [(-16,6.8),(-14.8,7.2),(-13.6,7.3)]:out.append(round_bale(x,y))
    out.append(cows())
    out.append('DEF START_PAD '+landing_pad(*S['start'],main=True))
    out.append('DEF GOAL_PAD '+landing_pad(*S['goals'][0],main=False))
    out.append(hay_obstacle())
    out.append(mavic(*S['start']))
    return '\n'.join(out)+'\n'

if __name__=='__main__':
    names=[('START_REALISTIC_FARM.wbt','overview'),
           ('DRONE_FOLLOW_FARM.wbt','follow'),
           ('FARM_TOP_VIEW.wbt','top')]
    for name,mode in names:
        dst=ROOT/'worlds'/name
        dst.write_text(scene(mode),encoding='utf8')
        print('Wrote',dst.name,round(dst.stat().st_size/1024/1024,2),'MB')
    print('Farm structures:',len(S['obstacles']),'crop fields:',len(S['fields']))
