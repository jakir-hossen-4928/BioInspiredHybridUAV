"""Pure Python hybrid gait, obstacle and support-force reference utilities."""
import math
from dataclasses import dataclass

LEG_NAMES=('fl','fr','rl','rr')

def gait_angles(phase, amplitude=0.26):
    # Diagonal-pair alternating trot; radians around each hip's local Y axis.
    wave=amplitude*math.sin(phase)
    return dict(zip(LEG_NAMES,(wave,-wave,-wave,wave)))


def force_estimate(mass_kg, grounded=True, feet=4):
    """Analytical weight-sharing estimate, NOT measured TouchSensor force."""
    force = float(mass_kg)*9.81/max(1,feet) if grounded else 0.0
    return {name:force for name in LEG_NAMES}

@dataclass(frozen=True)
class Detection:
    x: float
    y: float
    distance_m: float
    source: str


def projected_obstacle(position,yaw,distance_m,minimum=0.12,maximum=3.2):
    """Conservative front-sonar obstacle centroid in the local XY frame."""
    if not math.isfinite(distance_m) or not minimum<distance_m<maximum:return None
    x=position[0]+(distance_m+0.35)*math.cos(yaw)
    y=position[1]+(distance_m+0.35)*math.sin(yaw)
    return Detection(x,y,distance_m,'front ultrasonic')
