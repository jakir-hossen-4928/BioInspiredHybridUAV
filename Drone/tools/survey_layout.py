"""Build  spacious farm worlds for Webots R2025a with standard Python only.

Regenerate with: python tools/build_survey_world.py
Textures are bundled; no external PROTO, internet or Gazebo required.
"""
import math
import random
from pathlib import Path
import farm_details as cinematic

old = cinematic.old
ROOT = Path(__file__).resolve().parents[1]
random_generator = random.Random(14026)

# All dimensions below are meters (Webots ENU: z points up).

def camera(mode):
    if mode == 'follow':
        pos=(-57,-61,21); target=(-49,-47,.7)
        extras='follow "Drone" followType "Tracking Shot" followSmoothness 0.13'
        fov=.86
    elif mode == 'top':
        pos=(0,-1,151); target=(0,0,0)
        extras=''; fov=.94
    elif mode == 'closeup':
        pos=(-50.9,-50.2,1.45); target=(-49,-47,.28)
        extras='follow "Drone" followType "Tracking Shot" followSmoothness 0.08'
        fov=.88
    else:
        pos=(103,-113,107); target=(0,0,1.2)
        extras=''; fov=.97
    return ('Viewpoint { orientation %s position %s near 0.20 far 680 '
            'fieldOfView %.3f ambientOcclusionRadius 1.2 %s }' %
            (old.vec(cinematic.correct_look_at(pos,target)),old.vec(pos),fov,extras))

old.camera_block = camera


def wide_fence():
    parts=[]
    edge=57
    for i in range(-56,57,4):
        for x in (-edge,edge):
            parts.append(old.cube(x,i,.72,.15,.15,1.44,'wood'))
        for y in (-edge,edge):
            if y==-edge and i<-45:continue # entrance opening
            parts.append(old.cube(i,y,.72,.15,.15,1.44,'wood'))
    for x in (-edge,edge):
        for z in (.55,1.07):
            parts.append(old.cube(x,0,z,.10,114,.11,'wood'))
    for y in (-edge,edge):
        for z in (.55,1.07):
            if y==-edge:
                parts.append(old.cube(-1,y,z,53,.1,.10,'wood'))
                parts.append(old.cube(30,y,z,54,.1,.10,'wood'))
            else:
                parts.append(old.cube(0,y,z,114,.1,.10,'wood'))
    return 'Group { children [ '+ '\n'.join(parts) +' ] }'

old.fence_around = wide_fence


def perimeter_trees():
    parts=[]
    coords=[]
    for i in range(45):
        a=-55.0+(i%12)*9.6
        side=i%4
        if side==0: xy=(-60,a)
        elif side==1: xy=(60,a)
        elif side==2: xy=(a,-60)
        else: xy=(a,60)
        if math.dist(xy,(-49,-47))<7:continue
        coords.append(xy)
    for i,(x,y) in enumerate(coords):
        parts.append(cinematic.nature_tree(x,y,3.7+.18*(i%4),1000+i))
    return '\n'.join(parts)

cinematic.scenic_tree_line = perimeter_trees


def sparse_grass():
    rng=random.Random(197)
    quads=[]
    for i in range(1700):
        x=rng.uniform(-56,56);y=rng.uniform(-56,56)
        edge=max(abs(x),abs(y))
        patch=(-53<x<-40 and 12<y<32) or (43<x<56 and -15<y<15)
        if not (edge>46.5 or patch):continue
        if math.dist((x,y),(-49,-47))<3:continue
        w=.20+rng.random()*.3; h=.12+rng.random()*.39
        a=rng.random()*math.tau
        quads += [cinematic.card(x,y,.06,w,h,a),cinematic.card(x,y,.06,w,h,a+math.pi/2)]
    return cinematic.textured_mesh(quads,'wild_grass.png')

cinematic.weeds_grass = sparse_grass


def boundary_hedge():
    rng=random.Random(352)
    out=[]
    for i in range(76):
        y=-52+i*1.36
        for x in (-55.2,55.1):
            out.append(old.sphere(x+rng.uniform(-.15,.15),y,.25,.27+rng.random()*.19,'hedge'))
    return '\n'.join(out)

cinematic.grass_hedgerows = boundary_hedge


def distance_hills():
    # Flat farm until radius 65; hills only outside this ring, never in mission map.
    vertices=[];idx=[]
    nr=52
    for ri,r in enumerate((65,80,104,130,150)):
        for sector in range(nr):
            a=sector*math.tau/nr
            h=0 if ri==0 else (1.5+ri*.85)*(1.05+math.sin(3*a+1.2)+.25*math.sin(9*a))
            vertices.append((r*math.cos(a),r*math.sin(a),h-.10))
    for r in range(4):
        for k in range(nr):
            j=r*nr+k;n=r*nr+(k+1)%nr
            idx += [j,n,n+nr,-1,j,n+nr,j+nr,-1]
    point=' '.join('%.3f %.3f %.3f' % v for v in vertices)
    faces=' '.join(str(i) for i in idx)
    return ('Shape { appearance %s geometry IndexedFaceSet { creaseAngle .8 '
            'coord Coordinate { point [ %s ] } coordIndex [ %s ] } }' %
            (old.tex('grass.png',20,.98),point,faces))

cinematic.outer_terrain = distance_hills


def extended_roads():
    roads=[((-51,-48),(-21,-48),2.4),((-49,-47),(-22,-24),2.4),
           ((-24,-50),(-24,51),2.1),((-49,26),(50,26),2.1),
           ((23,-50),(23,48),2.1),((-49,-25),(50,-25),2.1),
           ((-49,12),(-31,12),1.7),((45,-50),(45,-22),1.9),
           ((-50,48),(45,48),2.0)]
    out=[]
    for a,b,w in roads:
        out.append(old.dirt_road(a,b,w))
        # Visual tyre tracks on straight road portions.
        dx=b[0]-a[0];dy=b[1]-a[1]
        length=math.hypot(dx,dy)
        angle=math.atan2(dy,dx)
        mx=(a[0]+b[0])/2;my=(a[1]+b[1])/2
        for off in (-.43,.43):
            out.append(old.cube(mx-math.sin(angle)*off,my+math.cos(angle)*off,
                                .058,length-.35,.13,.016,'mud',(0,0,1,angle)))
    return '\n'.join(out)


def photo_stations():
    parts=[]
    for ix,station in enumerate(old.S['photo_stops']):
        x,y=station['point']
        p=[old.cyl(0,0,.05,.47,.06,'stone'),
           old.cyl(0,0,.09,.31,.03,'water_blue'),
           old.cube(0,0,.111,.42,.06,.02,'white'),
           old.cube(0,0,.112,.06,.42,.02,'white')]
        name=('%02d - %s' %(ix+1,station['name'])).replace('"','')
        parts.append('Solid { translation %.2f %.2f 0 name "PHOTO STOP %s" children [ %s ] }'
                     %(x,y,name,'\n'.join(p)))
    return 'Group { children [ # All photo stops: ground markers\n'+'\n'.join(parts)+'\n] }'


def irrigation_extensions():
    parts=[]
    for x in (-39,-35,-31,-27):
        parts.append(old.cube(x,35,.09,.06,15.5,.05,'steel'))
        for y in (29.2,35.1,40.9):
            parts.append(old.cyl(x,y,.35,.035,.5,'metal'))
    for x in (29,34,39):
        parts.append(old.cube(x,-36,.09,.065,14.4,.075,'steel'))
        for y in (-41.0,-35.5,-29.9):
            parts.append(old.sphere(x,y,.16,.085,'water_blue'))
    return '\n'.join(parts)


def extra_equipment():
    out=[]
    # 2 extra tractors and a small weather mast in the spacious farm.
    out.append(old.cube(-49,-23,.6,2.3,1.1,1.2,'tractor'))
    out.append(old.cube(-49,-23,1.5,1.0,1.0,.60,'window'))
    for x in (-49.8,-48.2):
        for y in (-23.7,-22.3):
            out.append(old.cyl(x,y,.39,.40,.18,'wheel',(1,0,0,math.pi/2)))
    out.append(old.cyl(49,44,2.6,.15,5.2,'steel'))
    out.append(old.sphere(49,44,5.28,.33,'metal'))
    # Water reservoir at the northeastern service edge.
    out.append(old.cyl(50,-12,.24,2.2,.40,'water_blue'))
    out.append(old.cyl(50,-12,.05,2.43,.09,'stone'))
    return '\n'.join(out)


def create_world(mode):
    world=cinematic.create_world(mode)
    # Expand old (200m square) rolling terrain beneath expanded farm/outskirts.
    assert 'geometry Box { size 200.0000 200.0000 0.3200 }' in world
    world=world.replace('geometry Box { size 200.0000 200.0000 0.3200 }',
                        'geometry Box { size 310.0000 310.0000 0.3200 }',1)
    world=world.replace('boundingObject Box { size 200 200 .32 }',
                        'boundingObject Box { size 310 310 .32 }',1)
    world=world.replace('scale 75 75','scale 110 110')
    world=world.replace('CINEMATIC FARM ','FARM SURVEY ')
    world=world.replace('Cinematic Farm  - Farm Quadcopter A-Star',
                        'Drone - Farm Survey')
    world=world.replace('name "Farm terrain"', 'name "Spacious farm landscape"')
    # Extra fields are generated via scene.json (old.scene). Additional fine details:
    extension=('\n# Survey paths: roads, extra irrigation, waypoint/photo markers\n' +
               extended_roads()+'\n'+irrigation_extensions()+'\n'+extra_equipment()+
               '\n'+photo_stations()+'\n')
    assert 'DEF START_PAD ' in world
    world=world.replace('DEF START_PAD ',extension+'\nDEF START_PAD ',1)
    # The *actual* nadir Camera faces downward (Webots Camera optical axis is +X).
    # Rotation about local +Y by +pi/2 turns +X to -Z. Separate front camera preserved.
    assert 'Altimeter { name "altimeter" }' in world
    world=world.replace('Altimeter { name "altimeter" }',
        'Camera { name "survey_camera" translation 0.04 0 -0.17 '
        'rotation 0 1 0 1.57079632679 width 800 height 600 '
        'fieldOfView 1.16 near 0.08 far 130 }\n    Altimeter { name "altimeter" }',1)
    return world


def main():
    out=ROOT/'worlds'
    out.mkdir(exist_ok=True)
    for old_file in out.glob('*.wbt'):
        old_file.unlink()
    mode_names={'survey_start.wbt':'overview',
                'survey_follow.wbt':'follow',
                'survey_top.wbt':'top',
                'survey_closeup.wbt':'closeup'}
    for name,mode in mode_names.items():
        text=create_world(mode)
        (out/name).write_text(text,encoding='utf-8')
        print('Created', name,'%.2f MB' %(len(text.encode('utf-8'))/1e6))
    print('Stops:',len(old.S['photo_stops']),'missions:',len(old.S['missions']))

if __name__=='__main__':
    main()
