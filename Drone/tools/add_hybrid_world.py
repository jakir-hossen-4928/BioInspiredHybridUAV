"""Extend the existing Webots drone with four kinematically animated legs.

Run from Drone/: python tools/add_hybrid_world.py
Creates worlds/hybrid.wbt without modifying the six original farm worlds.
"""
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
WORLD=ROOT/'worlds'/'livo2.wbt'
TARGET=ROOT/'worlds'/'hybrid.wbt'


def leg_node(name,x,y):
    side=name.upper()
    material='PBRAppearance { baseColor 0.24 0.31 0.39 roughness 0.54 metalness 0.42 }'
    footmat='PBRAppearance { baseColor 0.75 0.47 0.22 roughness 0.71 metalness 0.16 }'
    return f'''DEF LEG_{side}_HIP Transform {{
  translation {x:.4f} {y:.4f} -0.020
  rotation 0 1 0 0
  children [
    Pose {{ translation 0 0 -0.145 children [
      Shape {{ appearance {material} geometry Box {{ size 0.085 0.085 0.290 }} }}
    ] }}
    DEF LEG_{side}_KNEE Transform {{
      translation 0 0 -0.295
      rotation 0 1 0 0
      children [
        Pose {{ translation 0 0 -0.115 children [
          Shape {{ appearance {material} geometry Box {{ size 0.074 0.074 0.230 }} }}
        ] }}
        TouchSensor {{
          translation 0 0 -0.228
          name "walk_force_{name}"
          type "force-3d"
          lookupTable []
          children [ Shape {{ appearance {footmat} geometry Box {{ size 0.165 0.128 0.045 }} }} ]
          boundingObject Box {{ size 0.165 0.128 0.045 }}
          physics Physics {{ density -1 mass 0.05 }}
        }}
      ]
    }}
  ]
}}
'''


def build():
    source=WORLD.read_text(encoding='utf8')
    assert source.count('DEF DRONE Robot {')==1
    assert source.count('controller "drone"')==1
    marker='] boundingObject Box { size 1.215 .765 .36 } physics Physics { mass 3.9 density -1 } }'
    assert source.count(marker)==1
    legs='\n# Articulated four-leg mobility teaching model\n'
    for leg,x,y in (('fl',.47,.35),('fr',.47,-.35),('rl',-.47,.35),('rr',-.47,-.35)):
        legs+=leg_node(leg,x,y)
    result=source.replace(marker,legs+marker)
    result=result.replace('controller "drone"','controller "hybrid"')
    result=result.replace('Drone - LiDAR IMU Camera Stream','Drone - Flight and Walking')
    result=result.replace('Fast LiDAR','Hybrid flight and walking')
    # Camera resolution and farm texture links are preserved byte-for-byte.
    for leg in ('FL','FR','RL','RR'):
        assert result.count('DEF LEG_'+leg+'_HIP Transform')==1
        assert result.count('DEF LEG_'+leg+'_KNEE Transform')==1
    assert result.count('type "force-3d"')==4
    TARGET.write_text(result,encoding='utf8')
    print('Generated',TARGET.name,'bytes',len(result),'4 legs, 4 force-3d sensors')
    return TARGET

if __name__=='__main__':build()
