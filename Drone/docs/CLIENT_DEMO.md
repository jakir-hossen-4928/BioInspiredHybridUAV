# Local client demonstration

Double-click `RUN_CLIENT_DEMO.bat` from the project root. The launcher
checks Python 3.9+, Webots R2025a, Docker and the `kiri` Docker context.
Docker must already be available. It starts the existing
`uav-liosam-nov27` container and expects its port 15882 on localhost.
It waits for the bridge and all four LIO-SAM nodes before opening Webots.

1. The normal farm controller runs the full survey. The launcher enables
   automatic rolling, walking and flight transitions, survey photos, return
   to home and landing. The farm controller is a kinematic simulation.
   LIO-SAM runs alongside it and its farm estimate is experimental.
2. After the farm stage, the launcher resets the ROS stack and opens the
   physics-driven arena. This stage uses live LIO-SAM for navigation and
   combined camera-LiDAR avoidance of a moving obstacle.

Both stages default to GUI real-time simulation. The farm sensor dashboard
opens automatically and refreshes every two seconds. The launcher console
shows sampled LiDAR, IMU, RGB, odometry and global-cloud counts for both
stages. If the running ROS container has `rviz2` and a working Linux display,
set `UAV_RVIZ_DISPLAY` to that display to open the live RViz view. The
launcher reports when RViz is unavailable.

Each launch writes a new `client-demo-sessions/<timestamp>/` with Webots
and ROS logs, farm photos and a final screenshot, arena frames and replan
screenshot, raw results and a session summary. The success exit code
requires measured rolling and walking, farm photos, a return and landing,
and a passing combined obstacle trial. The result files are the source
for what actually happened in a run.

Double-click `STOP_CLIENT_DEMO.bat` to request shutdown. It signals the
active launcher, which stops the Webots tree and the ROS container it
started. A container already in use is refused to protect the other job.
The launcher leaves the dashboard in the browser for inspection.

`RUN_CLIENT_DEMO.bat --mode fast` runs the same two worlds without real-time
pacing for local verification.
