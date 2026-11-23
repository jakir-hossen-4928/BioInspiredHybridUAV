# Wheel-legs patch quality and limits

- Source base: existing hybrid/ROS 2 Drone simulator.
- New files: `hybrid_wheels.wbt`, `hybrid_wheels_ros2.wbt`, wheel controller,
  configuration, batch launchers, generator, and unit tests.
- Offline test command: `python -m unittest discover -s tests -v`.
- Test result on the packaging environment: **59 offline tests passed**.
- Four wheels: each inserted inside the respective lower-leg Transform, with
  a colored hub and non-axisymmetric marker to make spinning visible.
- Wheel model: rigid visual Transform rotating under Supervisor commands.
- Four `force-3d` foot devices retained; raw force physics not verified.
- Ground motion: kinematic path following with turn-in-place and differential
  tire animation; not torque/friction physics.
- FAST-LIVO2 estimator: external ROS 2 component, not integrated/tested here.
- Not tested in this environment: Webots R2025a GUI, full ROS 2 runtime,
  FAST-LIVO2 odometry/map, measured wheel-ground contact dynamics.
