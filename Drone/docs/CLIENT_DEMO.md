# Local client demonstration

Double-click `RUN_CLIENT_DEMO.bat` from the project root. The launcher
checks Python 3.9+, Webots R2025a, Docker and the `kiri` Docker context.
Docker must already be available. It starts the existing
`uav-liosam-nov27` container and expects its port 15882 on localhost.
It waits for the bridge and all four LIO-SAM nodes before opening Webots.

1. The normal farm controller runs the full survey. The launcher enables
   automatic rolling, walking and flight transitions, survey photos, return
   to home and landing. After touchdown it follows a green wheel route to a
   nearby waypoint and parks. Insert or move a top-level Webots Solid with
   supported primitive collision geometry into the flight or ground route
   to trigger a replan. On the ground, the drone keeps its current target
   and holds if no safe path exists. The farm controller is a kinematic simulation.
   Its live geometry check uses the Webots scene. The arena uses sensor data.
   LIO-SAM runs alongside it and its farm estimate is experimental.
2. After the farm stage, the launcher resets the ROS stack and opens the
   physics-driven arena. This stage uses live LIO-SAM for navigation and
   combined camera-LiDAR avoidance of a moving obstacle.

Both stages default to GUI real-time simulation. The farm sensor dashboard
opens automatically and refreshes every two seconds. The launcher restores
two detached live camera windows side by side at every client launch. The farm uses `slam_camera`
for the front view and `survey_camera` for the bottom view. The arena
uses `oak_rgb` and `bottom_camera`. To reopen a closed window,
double-click its camera overlay in Webots.
The launcher console shows sampled LiDAR, IMU, RGB, odometry and global-cloud
counts for both stages. If the ROS container has `rviz2` and a working Linux display,
set `UAV_RVIZ_DISPLAY` to that display to open the live RViz view. The
launcher reports when RViz is unavailable.

Each launch writes a new `client-demo-sessions/<timestamp>/` with Webots
and ROS logs, farm photos and a final screenshot, arena frames and replan
screenshot, raw results and a session summary. The success exit code
requires measured rolling and walking, farm photos, a return and landing,
measured rolling after touchdown, and a passing combined obstacle trial.
The result files are the source
for what actually happened in a run.

Double-click `STOP_CLIENT_DEMO.bat` to request shutdown. It signals the
active launcher, which stops the Webots tree and the ROS container it
started. A container already in use is refused to protect the other job.
The launcher leaves the dashboard in the browser for inspection.
The stop console stays open so you can read whether it found an active run.
Windows releases the launcher lock when its process exits, so closing the
launcher window does not leave a stale session marker blocking later runs.

`RUN_CLIENT_DEMO.bat --mode fast` accelerates the farm survey for local
verification. The physics arena stays in real time to let ROS keep pace.
