# Client demonstration and validation report

## Demonstration

The Windows launcher checks Python, Webots R2025a, Docker and the Kiri
context. It starts the existing Humble container, waits for the bridge
and four LIO-SAM nodes, then opens the existing farm world and dashboard.
Its second stage resets ROS time and launches the physics-driven
combined-sensor arena. A separate BAT file requests shutdown.

The local GUI fast-mode test verified launcher startup, ROS readiness,
Webots farm execution and automatic dashboard opening. At 135.224
simulated seconds the live observer had recorded LiDAR, IMU, RGB,
odometry and global map data. The farm LIO-SAM estimate diverged during
this kinematic run. Control continued on the farm controller's normal
Webots pose. RViz was skipped because the running environment had no
configured Linux graphical display.

## Measured flight acceptance

| Scenario | Measured result |
|---|---|
| Repeated arena matrix | 18/18 passed at `71f31999c206fb6c0741bf888557decbd0c128d5`. All flights had zero contacts. Minimum body clearance was 0.973 m and maximum 3D goal error was 0.094 m. |
| Camera-only moving obstacle | Both repeats passed. Clearances were 1.663 m and 1.679 m, with 0.049 m and 0.054 m goal error and 22 and 19 replans. |
| Static arena map accuracy | Surface RMSE was 0.183 m and 0.111 m in two clouds. The actual recorded cloud was replayed and captured in RViz. |
| Physics-driven farm with LIO-SAM and altitude fusion | Zero contacts, 1.206 m configured clearance and 0.425 m goal error. The farm acceptance limit was 0.35 m. |
| Automated regression | 90 tests passed, including the original 82 and the launcher telemetry test. |

The arena uses Webots ground truth for evaluation. The farm physics trial
uses LIO-SAM and an onboard simulated altimeter for navigation. The
quantitative map scores describe measured surfaces in the static arena.

[Per-run measurements and RViz evidence](evidence/flight-matrix-v5/README.md) ·
[Professor's acceptance status](PROFESSOR_STATUS.md) ·
[Launcher usage](CLIENT_DEMO.md)
