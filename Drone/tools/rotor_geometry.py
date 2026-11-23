"""Scale drone geometry and generate four supervisor-driven visual rotors.

Rotors are visual elements; physical thrust is not modeled.
"""
import re
import math
from pathlib import Path
import drone_geometry as base

ROOT = Path(__file__).resolve().parents[1]
FLOAT = r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?'
SCALE = 1.5
ROTOR_DEFS = {
    'front left propeller': 'ROTOR_FL',
    'front right propeller': 'ROTOR_FR',
    'rear left propeller': 'ROTOR_RL',
    'rear right propeller': 'ROTOR_RR',
}


def format_number(num):
    s = ('%.5f' % num).rstrip('0').rstrip('.')
    if s == '-0': s = '0'
    if s.startswith('0.'): s=s[1:]
    if s.startswith('-0.'): s='-'+s[2:]
    return s


def match_close(text, start, op='{', cl='}'):
    assert text[start] == op
    depth = 0
    in_string = False
    escaped = False
    for i in range(start, len(text)):
        c = text[i]
        if c == '"' and not escaped:
            in_string = not in_string
        if not in_string:
            if c == op: depth += 1
            elif c == cl:
                depth -= 1
                if depth == 0: return i
        escaped = c == '\\' and not escaped
        if c != '\\': escaped = False
    raise ValueError('Unbalanced VRML node')


def scale_triples(line, field, root=False):
    pat = re.compile(r'(\b'+field+r'\s+)('+FLOAT+r')\s+('+FLOAT+r')\s+('+FLOAT+r')')
    def repl(match):
        a,b,c = [float(v) for v in match.groups()[1:]]
        xyz = (a,b,c*SCALE) if root else (a*SCALE,b*SCALE,c*SCALE)
        return match.group(1)+' '.join(format_number(v) for v in xyz)
    return pat.sub(repl,line)


def scale_primitive(line):
    # Avoid changing camera pixel width/height, LiDAR resolution, or sonar range.
    if 'geometry Sphere' in line and ' scale ' in (' '+line+' '):
        return line # Transform scale already grew the unit sphere
    if any(k in line for k in ('geometry Sphere','geometry Cylinder','boundingObject Sphere')):
        line=re.sub(r'(\bradius\s+)('+FLOAT+r')',
                    lambda m:m.group(1)+format_number(float(m.group(2))*SCALE),line)
    if 'geometry Cylinder' in line:
        line=re.sub(r'(\bheight\s+)('+FLOAT+r')',
                    lambda m:m.group(1)+format_number(float(m.group(2))*SCALE),line)
    return line


def replace_hinge_with_transform(text):
    result = text
    for motor_name, def_name in ROTOR_DEFS.items():
        probe = 'HingeJoint {'
        motor_loc = result.find('name "'+motor_name+'"')
        if motor_loc == -1:
            raise ValueError('Missing motor '+motor_name)
        start = result.rfind(probe, 0, motor_loc)
        if start == -1: raise ValueError('Joint before motor missing '+motor_name)
        open_curly = result.find('{', start)
        end = match_close(result, open_curly)
        joint = result[start:end+1]
        endpoint = re.search(r'\bendPoint\s+Solid\s*\{', joint)
        if endpoint is None: raise ValueError('No endpoint for '+motor_name)
        endpoint_text = joint[endpoint.end():]
        coordinates = re.search(r'\btranslation\s+('+FLOAT+r')\s+('+FLOAT+r')\s+('+FLOAT+r')',endpoint_text)
        if coordinates is None: raise ValueError('Missing rotor translation')
        x,y,z = [float(v) for v in coordinates.groups()]
        children_match = re.search(r'\bchildren\s*\[', endpoint_text)
        if children_match is None: raise ValueError('Missing rotor blades')
        children_open = endpoint_text.find('[',children_match.start())
        children_end = match_close(endpoint_text,children_open,'[',']')
        shapes = endpoint_text[children_open+1:children_end]
        # Give the second blade a slightly lighter finish, so spinning is visible.
        second = shapes.find('rotation 0 0 1 1.5707963')
        if second >= 0:
            p = shapes.find('baseColor 0.04 0.053 0.065',second)
            if p >= 0:
                shapes = (shapes[:p] + 'baseColor 0.54 0.58 0.62'
                          + shapes[p+len('baseColor 0.04 0.053 0.065'):])
        new = ('DEF %s Transform {\n'
               ' translation %s %s %s\n'
               ' rotation 0 0 1 0\n'
               ' children [%s]\n'
               '}') % (def_name, format_number(x),format_number(y),format_number(z),shapes)
        result = result[:start]+new+result[end+1:]
    assert 'HingeJoint {' not in result
    assert 'RotationalMotor {' not in result
    for rotor in ROTOR_DEFS.values():
        assert result.count('DEF '+rotor+' Transform {') == 1
    return result


def process_world(world):
    text=world.read_text(encoding='utf-8')
    start=text.find('DEF DRONE Robot {')
    if start<0:raise ValueError('Missing DRONE in '+str(world))
    open_curly=text.find('{',start)
    end=match_close(text,open_curly)
    block=text[start:end+1]
    scaled=[]
    for index,line in enumerate(block.splitlines(keepends=True)):
        line=scale_triples(line,'translation',root=(index==0))
        line=scale_triples(line,'anchor')
        line=scale_triples(line,'scale')
        line=scale_triples(line,'size')
        line=scale_primitive(line)
        scaled.append(line)
    enlarged=''.join(scaled)
    updated=replace_hinge_with_transform(enlarged)
    text=text[:start]+updated+text[end+1:]
    text=text.replace('FARM SURVEY','FARM SURVEY')
    # Keep the supplied bounding geometry. These will now agree with the enlarged body.
    assert text.count('DEF DRONE Robot {')==1
    return text


def main():
    base.main()
    worlds = ROOT/'worlds'
    for path in sorted(worlds.glob('*.wbt')):
        contents=process_world(path)
        new=path
        new.write_text(contents,encoding='utf-8')
        if new != path:
            path.unlink()
        print('Created',new.name,'rotors=4')
    assert len(list(worlds.glob('*.wbt'))) == 6

if __name__ == '__main__':
    main()
