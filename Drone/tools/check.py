"""Drone static package checks: all world files, assets, rotor nodes, and manual route JSON."""
from pathlib import Path
import json
import re
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'controllers'/'drone'))
from manual_routes import validate_manual_payload,check_safe_route

worlds=sorted((ROOT/'worlds').glob('*.wbt'))
assert len(worlds)==6, 'Expected six complete Webots world files'
scene=json.loads((ROOT/'controllers'/'drone'/'scene.json').read_text())
manual=json.loads((ROOT/'controllers'/'drone'/'manual_mission.json').read_text())
stops,config=validate_manual_payload(manual,scene)
legs,dist=check_safe_route(stops,scene,config)
assert len(scene['photo_stops'])==17
assert len(scene['missions'])==4
assert len(legs)==len(stops)+1
for f in worlds:
    txt=f.read_text(encoding='utf-8')
    assert txt.startswith('#VRML_SIM R2025a utf8')
    assert 'DEF DRONE Robot {' in txt
    assert 'name "Drone"' in txt
    assert 'name "survey_camera"' in txt
    assert 'name "camera"' in txt
    assert 'name "terrain_depth"' in txt
    assert 'RangeFinder {' in txt
    assert 'DEF ROTOR_FL Transform' in txt
    assert 'DEF ROTOR_FR Transform' in txt
    assert 'DEF ROTOR_RL Transform' in txt
    assert 'DEF ROTOR_RR Transform' in txt
    assert 'Mavic2Pro.proto' not in txt
    assert 'HingeJoint {' not in txt
    assert 'RotationalMotor {' not in txt
    assert ('width 3840 height 2160' if 'photos.wbt' == f.name else 'width 1920 height 1080') in txt
    for m in re.finditer(r'url\s*\[\s*"(\.\./textures/[^"\]]+)"\s*\]',txt):
        asset=(f.parent/m.group(1)).resolve()
        assert asset.is_file(),asset
print('Drone package checks: 6 worlds, local assets, 4 rotors, manual route valid, distance %.1f m' % dist)
