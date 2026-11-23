"""Create the farm worlds and retain a dated backup of the previous files."""
from datetime import datetime
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import rotor_geometry

DEPTH_NODE=('    # Depth sensor for landing assessment\n'
            '    RangeFinder { name "terrain_depth" translation 0 0 -0.38 '
            'rotation 0 1 0 1.57079632679 width 64 height 48 '
            'fieldOfView 0.70 near 0.05 minRange 0.08 maxRange 45 }\n')

TITLES={'farm.wbt':'Drone - Farm Survey', 'overview.wbt':'Drone - Farm Overview',
    'follow.wbt':'Drone - Follow View', 'closeup.wbt':'Drone - Close View',
    'top.wbt':'Drone - Top View', 'photos.wbt':'Drone - High Resolution Camera'}

def main():
    worlds=ROOT/'worlds'
    worlds.mkdir(exist_ok=True)
    backup=ROOT/'world_backups'/datetime.now().strftime('%Y%m%d_%H%M%S')
    backup.mkdir(parents=True,exist_ok=True)
    existing=list(worlds.glob('*.wbt'))
    for file in existing:shutil.copy2(file,backup/file.name)
    try:
        rotor_geometry.main()
        names=sorted(worlds.glob('*.wbt'))
        if len(names)!=6:raise RuntimeError('Expected six Webots world files')
        for file in names:
            text=file.read_text(encoding='utf-8')
            tag='    Altimeter { name "altimeter" }'
            if text.count(tag)!=1:raise RuntimeError('Altimeter placement not found')
            text=text.replace(tag,DEPTH_NODE+tag)
            text=text.replace('name "Autonomous Farm Quadcopter"','name "Drone"')
            text=text.replace('follow "Autonomous Farm Quadcopter"','follow "Drone"')
            text=text.replace('controller "farm_drone"','controller "drone"')
            text=text.replace('DEF STUDENT_DRONE Robot','DEF DRONE Robot')
            import re
            text=re.sub(r'WorldInfo \{ title "[^\"]+" info \[ "[^\"]+" \]',
                'WorldInfo { title "'+TITLES[file.name]+'" info [ "Farm survey using simulated sensors and known-map A* navigation with a kinematic flight model" ]',text,count=1)
            text=re.sub(r'(?m)^(\s*#.*?)\bV(?:6|7|8|9|10|11|12)\b',
                lambda m:re.sub(r'\bV(?:6|7|8|9|10|11|12)\b','',m.group(0)),text)
            # Preserve 4K front and survey camera dimensions on rebuild.
            text = re.sub(r'Camera\s*\{[^{}]*\}',
                lambda m: re.sub(r'\bheight\s+\d+', 'height 2160',
                    re.sub(r'\bwidth\s+\d+', 'width 3840', m.group(0))), text)
            file.write_text(text,encoding='utf-8')
    except Exception:
        for file in worlds.glob('*.wbt'):file.unlink()
        for file in backup.glob('*.wbt'):shutil.copy2(file,worlds/file.name)
        raise
    print('Six Webots worlds built successfully; previous files backed up in',backup)

if __name__=='__main__':main()
