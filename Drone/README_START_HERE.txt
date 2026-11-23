FINAL MERGED 4K DRONE PROJECT - WINDOWS WEBOTS R2025a
=========================================================

Bhai, yeh FULLY MERGED project hai - aapko alag ZIPs merge nahi karni.
Is folder mein original 4K farm textures, wheel-legged hybrid drone,
autonomous flight/walking/rolling demonstration, sensor bridge and ROS 2
source code saath hain.

START DEMO ON WINDOWS
1. Google Drive se poora "Drone_FINAL_4K_MERGED" folder DOWNLOAD karo.
2. Drive ki downloaded ZIP ko EXTRACT karo.
3. Andar Drone folder kholo.
4. RUN_DEMO.bat double-click karo, ya worlds/hybrid_wheels.wbt Webots mein kholo.
5. Webots mein Reset, phir Play dabao.
6. 3D scene par click kar ke controls use karo:
   T = wheels par roll    W = leg walking    I = flying
   D = obstacle detection    R = route replan   Y = terrain assessment

ROS 2 VERSION
- RUN_ROS2_DEMO.bat opens hybrid_wheels_ros2.wbt.
- ROS 2 bridge (Ubuntu 22.04 + Humble) needs to be running separately.
- Complete instructions: docs/ROS2_CONNECTION.md
- ROS 2 code: ros2_ws/src/drone_ros_bridge
- FAST-LIVO2 original third-party estimator is NOT embedded/installed here.
  Actual FAST-LIVO2 algorithm, calibration, maps and end-to-end ROS 2 testing
  have NOT been verified. See docs/FAST_LIVO2_VALIDATION.md.

SOURCE CODE
- OPEN_SOURCE_CODE.bat opens the project folder in VS Code (if installed).
- Main controllers/drone/drone.py, controllers/hybrid/hybrid.py,
  controllers/wheels/wheels.py, controllers/drone/hybrid_planner.py.

TESTING
- RUN_OFFLINE_TESTS.bat runs Python unittest test cases.
- Offline tests and ZIP integrity are NOT proof of actual Webots GUI,
  tire-ground friction, leg forces or ROS 2 estimator mapping correctness.
- Wheels, walking, flying are SUPERVISOR-KINEMATIC visual demonstrations.
- Four force-3d device nodes are included; their Newton outputs are not
  physically validated in the current kinematic simulation.

This delivery preserves original full-resolution textures without recompression.

SIMULATED LIVE CONSOLE
- Open Webots View > Console and run RUN_DEMO.bat.
- FAST-LIVO2 input feed, ROS2 TCP, hybrid planner, sensor and tire readings
  print with an explicit [SIMULATED] prefix every 2 simulated seconds.
- Status/log instructions: README_CONSOLE_DEMO.txt
