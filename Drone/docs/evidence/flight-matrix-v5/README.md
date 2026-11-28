# Flight matrix v5 acceptance evidence

## Acceptance criteria

Each arena flight must arrive with zero Webots contact steps, a strictly
positive minimum body-to-collider clearance and less than 0.35 m final
3D goal error. The trial must record at least one route change. Clearance
uses the collision box of the arena airframe and distances to the arena
floor, walls, landmarks and scenario obstacles. A detected contact sets the
minimum to zero. The camera, LiDAR and combined labels identify the
avoidance input. All three receive their navigation pose from LIO-SAM.

Each `run.json` identifies the source commit and run configuration. Raw
`result.json`, `track.csv` and `ros.jsonl` preserve the route decisions,
evaluation poses and ROS streams. The ROS observer relays
`/lio_sam/odometry/imu`. Farm trials that enable altitude fusion use the
onboard altimeter to correct slow vertical drift. Ground truth is used
for scoring.

## Repeated arena matrix

The [two complete repetitions](matrix-summary.csv) at
`71f31999c206fb6c0741bf888557decbd0c128d5` passed in all 18 runs.
The minimum logged clearance was 0.973 m and the largest final 3D goal
error was 0.094 m.

## 3D mapping

The map scorer uses the last `/lio_sam/mapping/map_global` cloud from
a static arena run and compares each finite point with the nearest
surface of 20 surveyed collision boxes. The transform translates the
declared launch LiDAR origin onto the world frame with no fitted
rotation, translation or scale. This measures the geometric accuracy
of observed points. The farm-wide map and unobserved surface coverage
remain unscored.

| Static camera run | Points | Surface RMSE (m) | P95 (m) | Max (m) | Within 0.25 m |
|---|---:|---:|---:|---:|---:|
| [First](71f3199-repeat-1/camera-static/map-accuracy.json) | 1,307 | 0.183 | 0.128 | 5.245 | 98.62% |
| [Second](71f3199-repeat-2/camera-static/map-accuracy.json) | 1,293 | 0.111 | 0.113 | 1.162 | 98.45% |

The [arena RViz capture](71f3199-repeat-1/camera-static/rviz.png) renders
the replayed cloud from the first scored flight. The [farm RViz
capture](farm-dense/run-2/rviz.png) shows the denser farm trial's last
published cloud.

## V4 camera-moving collision

The v4 run ended with one contact, zero clearance, 24 replans and a
3.495 m goal error. Its last replan occurred at 11.9 simulated seconds.
At 16.6 seconds, the logged clearance was 0.140 m and the approach speed
was approximately 0.70 m/s. The remaining time to contact was about
0.20 seconds at that speed. The old controller replaced an observation
only when it changed the route and lacked a stopping-distance check.

V5 maintains tracks on every observation cycle, budgets sensor and
odometry age when estimating the stopping corridor, and retains a
lateral swept footprint through camera dropout. It retains a blocked
goal and can move to a collision-free observation point during LiDAR
avoidance. Clearing a missed track requires returns beyond its footprint
on three consecutive observation cycles, with closer returns vetoing
the update. If pose or footprint uncertainty places the aircraft in the
expanded planning margin, the low-speed recovery is restricted to
increasing separation from those footprints.

## Earlier attempts

Qualification and intermediate attempts are retained alongside the
matrix. The earliest v5 qualification contacted an arena landmark and
the initial clearance scorer omitted that landmark. Subsequent
revisions added the surveyed arena fixtures to the planning and scoring
sets. An intermediate batch launched in a shared ROS domain suffered
cross-run topic interference. The acceptance batches use isolated ROS
domains and receiver ports.
