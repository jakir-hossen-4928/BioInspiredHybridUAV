"""Draw an explanatory farm route map (not a screenshot of the Webots application)."""
import json,sys,math
from PIL import Image, ImageDraw, ImageFont, ImageFilter
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
C=ROOT/'controllers'/'drone'
sys.path.insert(0,str(C))
from planner import Planner
scene=json.loads((C/'scene.json').read_text())
planner=Planner(scene['obstacles'],resolution=.5,boundary=scene['bounds_m'],safety=1.1)
route=planner.route(scene['start'],scene['goals'][0])
W,H=1420,1180
MAP_OX,MAP_OY=70,150
SIZE=988
SCALE=SIZE/60
img=Image.new('RGB',(W,H),(14,30,29))
d=ImageDraw.Draw(img,'RGB')
reg=Path('/usr/share/fonts/truetype/dejavu')
font1=ImageFont.truetype(str(reg/'DejaVuSans-Bold.ttf'),33)
font2=ImageFont.truetype(str(reg/'DejaVuSans.ttf'),18)
font3=ImageFont.truetype(str(reg/'DejaVuSans-Bold.ttf'),17)
font4=ImageFont.truetype(str(reg/'DejaVuSans.ttf'),13)
def xy(x,y):return (round(MAP_OX+SIZE/2+x*SCALE),round(MAP_OY+SIZE/2-y*SCALE))
def rect(x,y,sx,sy,fill,outline=None,width=1):
    xa,ya=xy(x-sx/2,y+sy/2);xb,yb=xy(x+sx/2,y-sy/2)
    d.rectangle((xa,ya,xb,yb),fill=fill,outline=outline,width=width)
def line(a,b,color,width=4):d.line([xy(*a),xy(*b)],fill=color,width=width)
d.rounded_rectangle((22,21,W-23,H-26),radius=22,fill=(21,36,37),outline=(55,76,70),width=2)
d.text((70,61),'DRONE / FARM MAP',font=font1,fill=(232,240,219))
d.text((72,104),'Farm world layout and obstacle-safe A* flight path  |  60 x 60 m farm',font=font2,fill=(151,187,169))
# Grass terrain mosaic.
terrain=Image.open(ROOT/'textures'/'grass.png').resize((SIZE,SIZE),Image.Resampling.BILINEAR)
terrain=terrain.convert('RGB')
img.paste(terrain,(MAP_OX,MAP_OY))
d=ImageDraw.Draw(img,'RGB')
# Tracks
for a,b,w in [((-23,-26),(-23,26),1.9),((-23,-.8),(23,-.8),2.1),((-23,-23),(25,-23),2.2),((23,-23),(23,-4),1.5)]:
    line(a,b,(154,115,72),int(w*SCALE))
    line(a,b,(191,157,101),2)
# Crop plots
for f in scene['fields']:
    color={'corn':(40,90,30),'wheat':(144,128,49),'lettuce':(38,100,53)}[f['kind']]
    rect(f['x'],f['y'],f['sx'],f['sy'],(91,63,42),(171,144,92),3)
    for row in range(f['rows']):
        xx=f['x']-f['sx']/2+.4+row*(f['sx']-.8)/(f['rows']-1)
        line((xx,f['y']-f['sy']/2+.3),(xx,f['y']+f['sy']/2-.3),color,5)
# Obstacles
colors={'barn':(166,66,49),'farmhouse':(210,192,150),'greenhouse':(142,202,193),'silo':(169,181,193),
        'water':(52,131,170),'shed':(148,114,82),'hay':(206,162,68),'orchard':(26,83,40),'tractor':(190,58,39)}
for ob in scene['obstacles']:
    if ob['kind']=='orchard':
        rect(ob['x'],ob['y'],ob['sx'],ob['sy'],(66,113,54),(178,196,135),2)
        for ix in range(3):
            for iy in range(3):
                p=xy(ob['x']-2.3+2.3*ix,ob['y']-2.3+2.3*iy)
                r=15;d.ellipse((p[0]-r,p[1]-r,p[0]+r,p[1]+r),fill=(30,89,40),outline=(183,176,64),width=2)
    else:
        rect(ob['x'],ob['y'],ob['sx'],ob['sy'],colors[ob['kind']],(229,220,186),3)
        if ob['kind']=='silo':
            p=xy(ob['x'],ob['y']);r=26;d.ellipse((p[0]-r,p[1]-r,p[0]+r,p[1]+r),fill=(190,196,198),outline=(99,123,144),width=3)
        if ob['kind']=='greenhouse':
            for rx in (-3,-1.5,0,1.5,3):
                aa=xy(ob['x']+rx,ob['y']-ob['sy']/2);bb=xy(ob['x']+rx,ob['y']+ob['sy']/2)
                d.line((aa,bb),fill=(232,244,236),width=2)
# Pond
p=xy(22,-16);r=round(2.5*SCALE)
d.ellipse((p[0]-r,p[1]-r,p[0]+r,p[1]+r),fill=(32,126,152),outline=(166,184,164),width=4)
# Fences
rect(0,0,56.5,56.5,None,(175,160,118),5)
# Route shadow, route
pts=[xy(*v) for v in route]
d.line(pts,fill=(15,39,36),width=16,joint='curve')
d.line(pts,fill=(87,247,204),width=7,joint='curve')
for i,p in enumerate(pts[1:-1],start=1):
    d.ellipse((p[0]-7,p[1]-7,p[0]+7,p[1]+7),fill=(246,252,239))
for tag,p,col in [('START',pts[0],(36,228,141)),('GOAL',pts[-1],(252,190,50))]:
    d.ellipse((p[0]-13,p[1]-13,p[0]+13,p[1]+13),fill=col,outline=(255,255,255),width=3)
    d.text((p[0]+19,p[1]-19),tag,font=font3,fill=(255,255,255),stroke_width=3,stroke_fill=(21,47,34))
# compass
cx,cy=1021,190
d.polygon([(cx,cy-31),(cx-8,cy+12),(cx,cy+6),(cx+8,cy+12)],fill=(247,241,203))
d.text((cx-7,cy+16),'N',font=font3,fill=(250,247,212))
# Sidebar
sx=1100
d.text((sx,174),'FARM FEATURES',font=font3,fill=(216,232,212))
labels=[('Corn rows',(38,95,43)),('Wheat crops',(177,149,57)),('Vegetable beds',(47,124,77)),
('Red barn',(166,66,49)),('Farmhouse',(210,192,150)),('Glass greenhouse',(142,202,193)),
('Grain silo',(169,181,193)),('Water tower',(52,131,170)),('Orange orchard',(24,94,45)),
('Equipment + tractor',(182,76,46)),('A* flight path',(87,247,204))]
for i,(lab,col) in enumerate(labels):
    yy=222+i*49
    d.rounded_rectangle((sx,yy,sx+25,yy+25),radius=4,fill=col)
    d.text((sx+42,yy+2),lab,font=font4,fill=(226,234,217))
d.text((sx,831),'ROUTE DETAILS',font=font3,fill=(216,232,212))
length=sum(math.dist(a,b) for a,b in zip(route,route[1:]))
for i,t in enumerate([f'Start:  {scene["start"]}',f'Goal:  {scene["goals"][0]}',f'Route:  {length:.1f} m',f'Waypoints: {len(route)}','Planner: 2D A* (known map)','Safety: 1.10 m']):
    d.text((sx,868+i*30),t,font=font4,fill=(175,211,190))
d.text((72,1125),'LAYOUT / ROUTE PREVIEW ONLY - Not a screenshot from Webots. Farm structures use local 3D geometry and PBR textures.',font=font4,fill=(145,164,152))
path=ROOT/'docs'/'screenshots'/'farm_plan.png'
img.save(path,optimize=True)
print(path, path.stat().st_size)
