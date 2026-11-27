"""Create wheel-equipped hybrid Webots demo worlds without changing originals.

Wheels are Transform-driven visuals mounted to each lower leg. The underlying
mobile robot remains Supervisor-kinematic, not physics-driven wheel traction.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORLDS = ROOT / 'worlds'
LEGS = ('fl', 'fr', 'rl', 'rr')
OAK_CAMERAS = '''    # OAK-D Pro OV9782 RGB and idealized OV9282 stereo depth output.
    Camera { name "oak_rgb" translation .72 0 .05 width 1280 height 800 fieldOfView 1.3962634 }
    Camera { name "oak_left" translation .72 .0375 .05 width 640 height 400 fieldOfView 1.3962634 }
    Camera { name "oak_right" translation .72 -.0375 .05 width 640 height 400 fieldOfView 1.3962634 }
    RangeFinder { name "oak_depth" translation .72 0 .05 width 96 height 60 fieldOfView 1.3962634
      near .10 minRange .20 maxRange 8 }
'''


def wheel_node(leg):
    side = leg.upper()
    sign = 1.0 if leg.endswith('l') else -1.0
    outer = 0.058 if sign > 0 else -0.058
    marker = 0.067 if sign > 0 else -0.067
    # The Webots ENU world has Z up and the drone rolls along local +X.
    # Cylinder height axis is Y, therefore the wheel spins around local Y.
    return f'''        # Off-road tire attached below the {side} lower leg
        DEF WHEEL_{side} Transform {{
          translation 0 {sign * .105:.3f} -0.140
          rotation 0 1 0 0
          children [
            Shape {{
              appearance PBRAppearance {{ baseColor 0.055 0.065 0.075 roughness 0.98 metalness 0.01 }}
              geometry Cylinder {{ radius 0.125 height 0.100 subdivision 28 }}
            }}
            Pose {{ translation 0 {outer:.3f} 0 children [
              Shape {{ appearance PBRAppearance {{ baseColor 0.37 0.43 0.48 roughness 0.36 metalness 0.65 }}
                      geometry Cylinder {{ radius 0.070 height 0.012 subdivision 24 }} }}
            ] }}
            Pose {{ translation 0 {marker:.3f} 0 children [
              Shape {{ appearance PBRAppearance {{ baseColor 0.95 0.48 0.11 roughness 0.34 metalness 0.42 }}
                      geometry Cylinder {{ radius 0.025 height 0.012 subdivision 20 }} }}
            ] }}
            Pose {{ translation 0.094 {marker:.3f} 0.030 children [
              Shape {{ appearance PBRAppearance {{ baseColor 0.98 0.65 0.21 roughness 0.49 metalness 0.1 }}
                      geometry Box {{ size 0.045 0.010 0.020 }} }}
            ] }}
            Pose {{ translation -0.076 {marker:.3f} -0.068 children [
              Shape {{ appearance PBRAppearance {{ baseColor 0.21 0.25 0.28 roughness 0.50 metalness 0.3 }}
                      geometry Box {{ size 0.030 0.010 0.026 }} }}
            ] }}
          ]
        }}
'''


def build():
    outputs = []
    for source_name, target_name in (
            ('hybrid.wbt', 'hybrid_wheels.wbt'),
            ('hybrid_ros2.wbt', 'hybrid_wheels_ros2.wbt')):
        src = (WORLDS / source_name).read_text(encoding='utf-8')
        assert src.count('DEF DRONE Robot {') == 1
        assert src.count('controller "hybrid"') == 1
        for leg in LEGS:
            needle = f'          name "walk_force_{leg}"'
            assert src.count(needle) == 1, (source_name, leg)
            pos = src.index(needle)
            idx = src.rfind('        TouchSensor {', 0, pos)
            assert idx > 0 and pos - idx < 160, leg
            src = src[:idx] + wheel_node(leg) + src[idx:]
        src = src.replace('controller "hybrid"', 'controller "wheels"', 1)
        camera_anchor = '    Camera { name "survey_camera"'
        assert src.count(camera_anchor) == 1
        src = src.replace(camera_anchor, OAK_CAMERAS + camera_anchor, 1)
        src = src.replace('Drone - Flight Walking ROS2',
                          'Drone - Flight Walking Wheels ROS2', 1)
        src = src.replace('Drone - Flight and Walking',
                          'Drone - Flight Walking Wheels', 1)
        assert src.count('DEF WHEEL_') == 4
        assert src.count('type "force-3d"') == 4
        assert src.count('width 3840 height 2160') >= 1
        assert src.count('controller "wheels"') == 1
        output = WORLDS / target_name
        output.write_text(src, encoding='utf-8', newline='\n')
        outputs.append(output)
    return outputs


if __name__ == '__main__':
    for path in build():
        print(path)
