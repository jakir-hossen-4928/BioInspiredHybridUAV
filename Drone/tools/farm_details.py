"""Webots R2025a cinematic farm world .
Enhances  farm with detailed organic vegetation, tracks, distant terrain,
photographic-style transparent foliage cards and correct camera orientation.
Self-contained geometry and textures; drone is a native Webots Robot.
The educational mission is supervisor-kinematic, not real quadrotor physics.
"""
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import farm_geometry as old

R = random.Random(20261121)


def correct_look_at(position, target):
    """VRML Viewpoint looks down local -Z, with local +Y camera up."""
    fx,fy,fz=(target[i]-position[i] for i in range(3))
    fmag=math.sqrt(fx*fx+fy*fy+fz*fz)
    fx,fy,fz=fx/fmag,fy/fmag,fz/fmag
    rx,ry,rz=fy,-fx,0.0
    rmag=math.hypot(rx,ry)
    rx,ry,rz=rx/rmag,ry/rmag,0.0
    ux,uy,uz=(ry*(-fz), (-rz*fx+rx*fz),rx*(-fy)+ry*fx)
    # up = cross(right,forward)
    ux=ry*fz-rz*fy
    uy=rz*fx-rx*fz
    uz=rx*fy-ry*fx
    # Matrix with camera local X=right, Y=up, Z=-forward (as columns).
    m=((rx,ux,-fx),(ry,uy,-fy),(rz,uz,-fz))
    trace=m[0][0]+m[1][1]+m[2][2]
    angle=math.acos(max(-1.0,min(1.0,(trace-1)/2)))
    if angle<1e-9:return (0,0,1,0)
    denom=2*math.sin(angle)
    if abs(denom)<1e-8:return (1,0,0,angle)
    return ((m[2][1]-m[1][2])/denom,
            (m[0][2]-m[2][0])/denom,
            (m[1][0]-m[0][1])/denom,angle)

old.look_at = correct_look_at


def cinematic_camera(mode):
    if mode == 'top':
        pos = (0, -2, 83)
        target = (0, 0, 0)
        extra = ''
        fov = 0.83
    elif mode == 'closeup':
        pos = (-25.5, -25.8, 1.55)
        target = (-23, -23, .25)
        extra = 'follow "Drone" followType "Tracking Shot" followSmoothness .10'
        fov = .82
    elif mode == 'follow':
        pos = (-32, -39, 13)
        target = (-20, -19, 2.5)
        extra = 'follow "Drone" followType "Tracking Shot" followSmoothness 0.16'
        fov = 0.84
    else:
        pos = (43, -54, 43)
        target = (0, 0, 1.5)
        extra = ''
        fov = 0.94
    return (f'Viewpoint {{ orientation {old.vec(correct_look_at(pos,target))} position {old.vec(pos)} '
            f'near 0.20 far 350 fieldOfView {fov} ambientOcclusionRadius 1.25 {extra} }}')

old.camera_block = cinematic_camera
old.MAT['dark_soil'] = old.col((.19,.105,.072), .98)
old.MAT['mud'] = old.tex('muddy_ruts.png', 4, .98)
old.MAT['weathered_wood'] = old.tex('farm_wood.png', 3, .94)
old.MAT['stone_light'] = old.col((.63,.64,.59),.96)
old.MAT['hedge'] = old.col((.19,.36,.17),.97)
old.MAT['solar'] = old.col((.045,.105,.19),.13,.45)


def textured_mesh(quads, texture, double_sided=True):
    """Draw many alpha-masked vegetation cards in one efficient Webots Shape."""
    if not quads:
        return ''
    points = []
    indices = []
    for q in quads:
        i = len(points)
        points.extend(q)
        indices.extend([i,i+1,i+2,i+3,-1])
    point = ' '.join('%.3f %.3f %.3f' % tuple(p) for p in points)
    idx = ' '.join(str(x) for x in indices)
    uv = ' '.join('0 0 1 0 1 1 0 1' for _ in quads)
    tidx = ' '.join('0 1 2 3 -1' for _ in quads)
    # Four texture coordinates are reused, minimizing scene size.
    return ('Shape { appearance PBRAppearance { baseColorMap ImageTexture { '
            'url [ "../textures/%s" ] repeatS FALSE repeatT FALSE } '
            'metalness 0 roughness 1 } geometry IndexedFaceSet { '
            'coord Coordinate { point [ %s ] } coordIndex [ %s ] '
            'texCoord TextureCoordinate { point [ 0 0, 1 0, 1 1, 0 1 ] } '
            'texCoordIndex [ %s ] } }' % (texture, point, idx, tidx))


def card(x,y,z, width,height, angle):
    # Plane is vertical with width in the horizontal plane, base at Z=z.
    dx = math.cos(angle) * width/2
    dy = math.sin(angle) * width/2
    return [(x-dx,y-dy,z),(x+dx,y+dy,z),
            (x+dx,y+dy,z+height),(x-dx,y-dy,z+height)]


def branch(p1, p2, radius, material='darkwood'):
    """Orient a cylinder between two 3D points without numeric packages."""
    dx,dy,dz=(p2[i]-p1[i] for i in range(3))
    length=math.sqrt(dx*dx+dy*dy+dz*dz)
    ax,ay=-dy,dx
    norm=math.hypot(ax,ay)
    if norm<1e-9:
        rot=None if dz>0 else (1,0,0,math.pi)
    else:
        rot=(ax/norm,ay/norm,0,math.acos(max(-1,min(1,dz/length))))
    mid=tuple((p1[i]+p2[i])/2 for i in range(3))
    return old.cyl(mid[0],mid[1],mid[2],radius,length,material,rot)


def nature_tree(x,y,height=4.0,seed=1,fruit=False):
    rng=random.Random(seed)
    parts=[]
    trunk_h=.59*height
    p0=(x,y,.1)
    p1=(x,y,trunk_h)
    parts.append(branch(p0,p1,.14*height/4,'darkwood'))
    foliage=[]
    # Asymmetric branch network, irregular leafy canopy, not cartoon spheres.
    for j in range(7):
        ang=j*math.tau/7 + rng.uniform(-.17,.17)
        r=(.82+rng.uniform(-.12,.35))*height/4
        z=(.56+.07*rng.random())*height
        tip=(x+r*math.cos(ang),y+r*math.sin(ang),height*(.73+rng.random()*.09))
        parts.append(branch((x,y,z),tip,.045*height/4,'darkwood'))
        for k in range(5):
            cx=tip[0]+rng.uniform(-.75,.75)*height/4
            cy=tip[1]+rng.uniform(-.75,.75)*height/4
            base_z=tip[2]+rng.uniform(-.25,.1)*height/4
            width=(1.0+rng.random()*.7)*height/4
            ht=(.95+rng.random()*.7)*height/4
            foliage.append(card(cx,cy,base_z,width,ht,ang+k*.7))
    # Inner canopy and crown fill.
    for k in range(14):
        a=rng.random()*math.tau
        rr=rng.random()*.7*height/4
        cx=x+math.cos(a)*rr
        cy=y+math.sin(a)*rr
        zz=(.59+.31*rng.random())*height
        foliage.append(card(cx,cy,zz,
                            (1.15+rng.random()*.65)*height/4,
                            (.9+rng.random()*.5)*height/4,a+.5))
    parts.append(textured_mesh(foliage,'orange_foliage.png' if fruit else 'real_leaves.png'))
    if fruit:
        for k in range(7):
            a=rng.random()*math.tau
            rr=(.48+rng.random()*.31)*height/4
            zz=(.68+rng.random()*.20)*height
            parts.append(old.sphere(x+math.cos(a)*rr,y+math.sin(a)*rr,zz,.075,'orange'))
    return '\n'.join(parts)


def orchard_realistic(ob):
    parts=[old.cube(ob['x'],ob['y'],.022,ob['sx'],ob['sy'],.046,'grass')]
    for row in range(3):
        for ix in range(3):
            x=ob['x']+(-2.35+ix*2.35)
            y=ob['y']+(-2.4+row*2.35)
            parts.append(nature_tree(x,y,3.15+.2*((ix+row)%2),121+row*3+ix,True))
    return '\n'.join(parts)

old.orchard=orchard_realistic


def scenic_tree_line():
    parts=[]
    coords=[]
    # Large shelterbelts outside the navigable field boundary.
    for k in range(28):
        side=k%4
        a=(-24+((k*11)%49))
        if side==0: x=-29.8;y=a
        elif side==1: x=29.8;y=a
        elif side==2: x=a;y=-29.9
        else: x=a;y=29.9
        if abs(x+23)<2.5 and y < -23: continue
        coords.append((x,y))
    for i,(x,y) in enumerate(coords):
        parts.append(nature_tree(x,y,3.8+(i%5)*.22,101+i))
    return '\n'.join(parts)


def weeds_grass():
    rng=random.Random(477)
    quads=[]
    for i in range(920):
        x=rng.uniform(-27.8,27.8)
        y=rng.uniform(-27.8,27.8)
        # Grass tufts belong along boundaries, paths, and unplanted corners.
        close_to_edge=abs(x)>24.6 or abs(y)>24.6
        patch = (-27<x<-19 and 3<y<11) or (15<x<26 and -19<y<-5)
        if not (close_to_edge or patch):continue
        # Preserve takeoff and goal pads.
        if math.hypot(x+23,y+23)<2.7 or math.hypot(x-23,y-23)<2.7:continue
        width=.22+rng.random()*.45
        ht=.17+rng.random()*.43
        rot=rng.random()*math.tau
        quads.extend([card(x,y,.055,width,ht,rot),card(x,y,.055,width,ht,rot+math.pi/2)])
    return textured_mesh(quads, 'wild_grass.png')


def gravel_and_tracks():
    roads=[((-28,-23),(-20,-23),2.3),((-23,-26),(-23,26),1.85),
           ((-23,-.8),(23,-.8),2.05),((-23,-23),(25,-23),2.2),
           ((23,-23),(23,-4),1.7),((-1,-23),(-1,-17),1.35),
           ((6,2),(6,9),1.3)]
    out=[]
    for a,b,w in roads:
        dx=b[0]-a[0];dy=b[1]-a[1]
        length=math.hypot(dx,dy)
        angle=math.atan2(dy,dx)
        mx=(a[0]+b[0])/2;my=(a[1]+b[1])/2
        for off in (-w*.30,w*.30):
            x=mx-math.sin(angle)*off;y=my+math.cos(angle)*off
            out.append(old.cube(x,y,.065,length,.18,.016,'mud',(0,0,1,angle)))
    # One isolated muddy puddle on a farm service track.
    out.append(old.pose(-23,4,.068, [old.box(1.15,.69,.020,'water_blue')],(0,0,1,.12)))
    return '\n'.join(out)


def grass_hedgerows():
    rng=random.Random(141)
    out=[]
    # Irregular low bushes, around the back of farm, not in flight route.
    for i in range(56):
        y= -26 + i*.95
        x=27.5+rng.uniform(-.18,.18)
        out.append(old.sphere(x,y,.38,.46+rng.random()*.16,'hedge'))
        if i%3==0:
            out.append(old.sphere(-27.4,y,.40,.35+rng.random()*.15,'leaf3'))
    return '\n'.join(out)


def farm_details():
    p=[]
    # Barn timber, sliding door ironwork, exterior hay piles.
    for yy in (-.15,.45,1.05):
        p.append(old.cube(-10.5-4.1,3+yy,1.4,.06,.07,2.4,'wood'))
    # Farmhouse stoop with stepping stones.
    for i in range(4):
        p.append(old.cube(-17,15-3.6-i*.7,.045,1.05,.46,.09,'stone_light'))
    # Tractor and irrigation parking area: fuel drum, crates and pallet.
    for i in range(3):
        p.append(old.cyl(-9.1+i*.65,-22.0,.46,.28,.9,'roof_red'))
        p.append(old.cyl(-9.1+i*.65,-22.0,.91,.29,.06,'steel'))
    for i in range(3):
        p.append(old.cube(-1.9+i*.6,-22.6,.35,.48,.52,.65,'weathered_wood'))
    # Solar panels on top of shed, characteristic modern farm accessory.
    for i in range(3):
        p.append(old.cube(-5.7+i*1.38,-11.8,5.00,1.12,1.85,.035,'solar',(0,1,0,.1)))
        p.append(old.cube(-5.7+i*1.38,-11.8,4.98,1.2,1.94,.06,'steel',(0,1,0,.1)))
    # Bird boxes, sacks, coiled hose, hose faucets and gravel scatter.
    for i in range(6):
        p.append(old.cube(12.7+i*.31,8.5,.23,.32,.31,.44,'hay'))
    rng=random.Random(71)
    for _ in range(60):
        x=rng.uniform(-25,25)
        y=rng.choice([-1.15,-.85,-23.9,-22.1])+rng.uniform(-.25,.25)
        if (x+23)**2+(y+23)**2<2:continue
        p.append(old.sphere(x,y,.042,rng.uniform(.035,.12),'stone'))
    # Decorative plot signs placed near planted fields.
    for x,y,txt in [(-15,-17,'CORN'),(7,-21,'WHEAT'),(-1,10,'VEGETABLES')]:
        p.append(old.cube(x,y,.75,.07,.07,1.4,'wood'))
        p.append(old.cube(x,y,1.57,1.50,.09,.44,'weathered_wood'))
        # Sign is not text geometry because Webots requires FontStyle; keep sign simple.
    return '\n'.join(p)


def outer_terrain():
    """Rolling hills only outside farm; flat inner area preserves planner."""
    points=[];ids=[];rng=random.Random(8)
    for ring,rad in enumerate((32.5,44,59,79,100)):
        for sector in range(56):
            a=sector*math.tau/56
            x=rad*math.cos(a);y=rad*math.sin(a)
            # Smooth distant hills in green/dry gradients.
            wave=(math.sin(3*a+.7)+1)*.36+(math.sin(9*a+1.6)+1)*.13
            height=(0 if ring==0 else (1.0+ring*1.4)*wave)
            points.append((x,y,height-.08))
    for ring in range(4):
        for sector in range(56):
            j=ring*56+sector;k=ring*56+(sector+1)%56
            p=j+56;q=k+56
            ids.extend([j,k,q,-1,j,q,p,-1])
    xyz=' '.join('%.2f %.2f %.2f'%t for t in points)
    ix=' '.join(map(str,ids))
    return (f'Shape {{ appearance {old.tex("grass.png",22,.99)} geometry IndexedFaceSet {{ '
            f'creaseAngle 1.0 coord Coordinate {{ point [ {xyz} ] }} '
            f'coordIndex [ {ix} ] }} }}')


def create_world(mode):
    world = old.scene(mode)
    # Use genuine blue daytime sky. Toned-down sun improves PBR and shadows.
    world=world.replace('DirectionalLight { direction -0.55 -0.45 -1 intensity 2.2 ambientIntensity .58 castShadows TRUE }',
                        'DirectionalLight { direction -0.35 -0.48 -0.80 color 1 0.955 0.85 intensity 1.9 ambientIntensity 0.55 castShadows TRUE }')
    world=world.replace('Background { skyColor [ 0.56 0.73 0.90 ] }',
                        'Background { skyColor [ 0.66 0.80 0.95 ] }')
    # Extend terrain so the farm sits in landscape rather than a 64 m floating square.
    world=world.replace('Farm terrain" children [ '+old.box(64,64,.32,'grass'),
                        'Farm terrain" children [ '+old.box(200,200,.32,'grass'))
    world=world.replace('boundingObject Box { size 64 64 .32 }',
                        'boundingObject Box { size 200 200 .32 }')
    world=world.replace('url [ "../textures/grass.png" ] } textureTransform TextureTransform { scale 18 18 }',
                        'url [ "../textures/grass.png" ] } textureTransform TextureTransform { scale 75 75 }')
    additions=[outer_terrain(),scenic_tree_line(),weeds_grass(),grass_hedgerows(),
               gravel_and_tracks(),farm_details()]
    assert 'DEF START_PAD ' in world
    world=world.replace('DEF START_PAD ', '\n# CINEMATIC FARM : organic hedgerows / tracks / shelterbelts / hills\n'+'\n'.join(additions)+'\nDEF START_PAD ', 1)
    world=world.replace('Farm Autonomous Navigation - Local Farm Quadcopter',
                        'Cinematic Farm  - Farm Quadcopter A-Star')
    return world


def main():
    texdir=ROOT/'textures'
    texdir.mkdir(exist_ok=True)
    if "--textures" in sys.argv or not all((texdir / n).is_file() for n in ("real_leaves.png","orange_foliage.png","wild_grass.png","muddy_ruts.png")):
        from PIL import Image, ImageDraw
        rng=random.Random(4321)
        def foliage_texture(name,main,details,kind='leaves'):
            im=Image.new('RGBA',(512,512),(0,0,0,0))
            d=ImageDraw.Draw(im,'RGBA')
            if kind=='grass':
                for _ in range(135):
                    x=rng.uniform(120,390);bottom_y=500
                    dx=rng.uniform(-140,140)
                    h=rng.uniform(95,440)
                    width=rng.uniform(3,10)
                    green=rng.choice(main)
                    d.polygon([(x-width, bottom_y),(x+width,bottom_y),(x+dx/2, 500-h*.55),(x+dx,500-h)],fill=green)
                    if rng.random()<.19:
                        d.ellipse((x+dx-2,500-h-4,x+dx+4,500-h+13),fill=rng.choice(details))
            else:
                # Organic tree silhouette of ~100 leaves with holes and branch veins.
                for _ in range(540):
                    a=rng.random()*math.tau;r=(rng.random()**.62)*217
                    cx=256+math.cos(a)*r*1.04;cy=273+math.sin(a)*r*.92
                    if not (20<cx<495 and 20<cy<490):continue
                    w=rng.uniform(7,19)
                    h=rng.uniform(5,14)
                    c=rng.choice(main)
                    d.ellipse((cx-w,cy-h,cx+w,cy+h),fill=c)
                    if rng.random()<.24:
                        d.ellipse((cx-w*.5,cy-h*.5,cx+w*.25,cy+h*.15),fill=rng.choice(details))
                for _ in range(35):
                    x=rng.randint(80,430);y=rng.randint(85,436)
                    d.line([(x,y),(x+rng.randint(-6,6),y+rng.randint(-8,8))],fill=(38,55,27,120),width=1)
            im.save(texdir/name)
        foliage_texture('real_leaves.png',[(39,90,34,253),(63,119,45,250),(74,133,48,248),(44,98,35,246),(56,111,39,254)],[(103,148,62,245),(25,76,24,240)])
        foliage_texture('orange_foliage.png',[(32,80,26,255),(51,111,33,248),(67,130,39,250),(42,103,32,253)],[(103,144,52,245),(27,66,23,245)])
        foliage_texture('wild_grass.png',[(43,84,28,235),(65,106,40,245),(86,123,49,229),(116,131,61,225)],[(190,155,87,240),(180,197,111,240)],'grass')
        # Mud with repeated irregular dark tire marks. RGB only: no alpha.
        im=Image.new('RGB',(512,512),(123,94,65))
        px=im.load()
        import math as m
        for yy in range(512):
            for xx in range(512):
                n=rng.randrange(-22,23)
                ridges=18*m.sin(xx/11.0)+10*m.sin(yy/19.0)
                track=-33 if (80<xx<150 or 352<xx<420) else 0
                b=n+int(ridges)+track
                px[xx,yy]=(max(15,min(255,123+b)),max(12,min(255,94+b)),max(9,min(255,65+b)))
        im.save(texdir/'muddy_ruts.png',optimize=True)
    modes={'START_CINEMATIC_FARM.wbt':'overview',
           'DRONE_FOLLOW_CINEMATIC.wbt':'follow',
           'CINEMATIC_TOP_VIEW.wbt':'top',
           'DRONE_CLOSEUP_TEST.wbt':'closeup'}
    for fname,mode in modes.items():
        file=ROOT/'worlds'/fname
        file.write_text(create_world(mode),encoding='utf8')
        print('Created',fname,round(file.stat().st_size/1024/1024,2),'MB')
    print('Added 3 organic textures + mud tracks, vegetation, hills and correct viewpoint.')

if __name__=='__main__':
    main()
