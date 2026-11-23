WEBOTS SIMULATED LIVE CONSOLE - INSTRUCTIONS
============================================

1. Open the Drone folder.
2. Double click RUN_DEMO.bat (or open worlds/hybrid_wheels.wbt in Webots).
3. In Webots, use View -> Console if the Console pane is hidden.
4. Press Reset, then Play.
5. Webots controller prints the live technical console every 2 simulated seconds.
6. Use keys T (wheels), W (walk), I (fly), D (obstacle sensing).
7. Console readings are prefixed [SIMULATED]. This labels source provenance.

READINGS
- GPS, IMU, gyro, acceleration, sonar, altitude, LiDAR are read from Webots
  simulated devices when present; unavailable/invalid readings appear N/A.
- Foot force-3d sensor numbers are shown as unvalidated Webots values only.
  Support estimate is clearly a model, not a force sensor measurement.
- Wheel rotation, commanded path, mode and state come from kinematic controller.
- ROS 2 TCP link shows network connection and sent/dropped counts only.
  TCP connected does not prove ROS 2 topics or FAST-LIVO2 estimation works.
- FAST-LIVO2 estimator, localization and SLAM map remain NOT VERIFIED.
- 'Hybrid A*' here means 8-heading, mode-augmented educational grid A*, not
  the paper's full research continuous-curvature algorithm.

FILES SAVED DURING A RUN
Drone/captures/<session>/CONSOLE_LIVE_STATUS.txt
Drone/captures/<session>/CONSOLE_TELEMETRY.jsonl

To disable the verbose print, change console_live_enabled to false in
controllers/drone/settings.json.

OFFLINE VALIDATION
From Drone folder run RUN_OFFLINE_TESTS.bat, or:
python -m unittest discover -s tests -v

Actual Webots GUI, ROS 2 node health and estimator outputs must be separately
verified on the user's computer. These tests are not proof of real SLAM output.
