# Wheel-legged drone demonstration

For setup and controls see `docs/WHEEL_LEGS_DEMO.md`.

Start `hybrid_wheels.bat` to demonstrate autonomous rolling -> aerial flight.
The alternative `hybrid_wheels_ros2.bat` keeps the existing sensor bridge, but
an external Ubuntu ROS 2 process is required to publish and use ROS topics.

This is a visual and kinematic ground-drive prototype, not physics-driven
traction or a verified FAST-LIVO2 SLAM implementation.
