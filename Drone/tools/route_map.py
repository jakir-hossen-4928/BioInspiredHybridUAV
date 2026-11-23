"""Preview image of the survey PLAN (not a Webots 3D screenshot)."""
import json
import math
import sys
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'controllers'/'drone'))
from planner import Planner
s=json.loads((ROOT/'controllers'/'drone'/'scene.json').read_text())
stop_by={e['id']:e for e in s['photo_stops']}
mode='FULL_FARM_SURVEY'
selected=[stop_by[key] for key in s['missions'][mode]]
planner=Planner(s['obstacles'],resolution=.75,boundary=s['bounds_m'],safety=1.2)
W,H=1700,1240
im=Image.new('RGB',(W,H),(15,28,29))
d=ImageDraw.Draw(im)
try:
    bold=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',36)
    regular=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',22)
    small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',17)
    tiny=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',14)
except OSError:
    bold=regular=small=tiny=ImageFont.load_default()
d.text((72,34),'DRONE / FARM ROUTE',font=bold,fill=(235,246,236))
d.text((73,91),'17 photography stops   /   A* routes   /   4 survey missions   /   return to home',font=regular,fill=(149,186,165))
origin=(90,185)
unit=8.55

def p(x,y):
    return (round(origin[0]+(x+58)*unit),round(origin[1]+(58-y)*unit))

def rect(cx,cy,sx,sy,fill,outline=None):
    x0,y0=p(cx-sx/2,cy+sy/2);x1,y1=p(cx+sx/2,cy-sy/2)
    d.rectangle((x0,y0,x1,y1), fill=fill,outline=outline,width=2)

def ellipse(x,y,r,color,outline=None,width=1):
    px,py=p(x,y)
    d.ellipse((px-r,py-r,px+r,py+r),fill=color,outline=outline,width=width)

rect(0,0,114,114,(43,77,48),(123,154,102))
# service roads
for road in [((-49,-47),(-22,-24)),((-24,-50),(-24,51)),((-49,26),(50,26)),
             ((23,-50),(23,48)),((-49,-25),(50,-25)),((-50,48),(45,48)),
             ((-23,-23),(23,-23)),((-23,-1),(23,-1))]:
    d.line((p(*road[0]),p(*road[1])),fill=(130,113,73),width=14)
    d.line((p(*road[0]),p(*road[1])),fill=(194,171,119),width=3)
for field in s['fields']:
    col={'corn':(57,137,50),'wheat':(174,145,65),'lettuce':(43,109,72)}[field['kind']]
    rect(field['x'],field['y'],field['sx'],field['sy'],col,(194,183,116))
    x0=field['x']-field['sx']/2+.5
    n=int(field.get('rows',12))
    for k in range(n):
        x=x0+k*(field['sx']-1)/max(1,n-1)
        d.line((p(x,field['y']-field['sy']/2+.3),p(x,field['y']+field['sy']/2-.3)),
               fill=(95,84,45) if field['kind']=='wheat' else (35,96,52),width=2)
for ob in s['obstacles']:
    col=(125,56,52) if ob['kind']=='barn' else (110,115,130)
    if ob['kind']=='orchard': col=(28,82,54)
    rect(ob['x'],ob['y'],ob['sx'],ob['sy'],col,(236,222,183))
# Preview a fully collision-safe A* route.
cur=s['start']
route=[tuple(cur)]
for station in selected+[{'point':s['start']}]:
    nxt=station['point']
    leg=planner.route(cur,nxt)
    route.extend(leg[1:])
    cur=nxt
coords=[p(*x) for x in route]
d.line(coords,fill=(8,35,38),width=12,joint='curve')
d.line(coords,fill=(78,228,238),width=6,joint='curve')
# first active leg
first=planner.route(s['start'],selected[0]['point'])
d.line([p(*i) for i in first],fill=(255,221,89),width=8)
for ix,item in enumerate(selected):
    x,y=item['point'];pixel=p(x,y)
    ellipse(x,y,15,(254,193,80),(27,38,33),3)
    text=str(ix+1)
    box=d.textbbox((0,0),text,font=tiny)
    d.text((pixel[0]-(box[2]-box[0])/2,pixel[1]-(box[3]-box[1])/2-1),text,font=tiny,fill=(21,28,25))
# home station
ellipse(*s['start'],20,(48,235,135),(232,246,230),3)
px,py=p(*s['start']);d.text((px+16,py+18),'HOME',font=small,fill=(214,250,228))
# side itinerary panel
x=1130
size=(520,970)
d.rounded_rectangle((x,183,x+510,1170),radius=25,fill=(26,51,43),outline=(71,111,90),width=3)
d.text((x+25,208),'17 SURVEY PHOTO STOPS',font=regular,fill=(245,244,218))
for i,item in enumerate(selected):
    color=(232,203,121) if i%2==0 else (206,236,220)
    d.text((x+28,260+i*46),f'{i+1:02d}  {item["name"][:28]}',font=small,fill=color)
d.text((89,1192),'Top-down planning diagram (not an actual Webots GUI render)  •  Coordinates in meters',font=small,fill=(157,176,166))
im.save(ROOT/'docs'/'screenshots'/'route_plan.png',optimize=True)
print(ROOT/'docs'/'screenshots'/'route_plan.png')
