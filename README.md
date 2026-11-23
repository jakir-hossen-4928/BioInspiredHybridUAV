# Bio-Inspired Hybrid UAV in Webots

The LaTeX technical report and its source are stored locally in `LaTeX/BioInspiredHybridUAV-Documentation/`.

This Windows project implements the agricultural navigation simulation in Section 4.3 of the [design report](Bio-Inspired%20Hybrid%20UAV.pdf). It adds animated leg and wheel modes, terrain screening and an optional simulated sensor stream. The navigation demonstration uses a Webots Supervisor to move a kinematic robot around a known farm map.

## Run on Windows

1. Keep `worlds/`, `controllers/` and `textures/` together inside `Drone/`. If setting up from the original archives, extract all three into the same parent folder so they share one `Drone/` tree. The assembled working copy already includes fixes.
2. Use Webots R2025a and Python 3. Run `python --version` in PowerShell. If Webots cannot start the controller, set its Python command in **Tools → Preferences → General** to the installed Python executable and reopen the world.
3. Double-click [RUN_DEMO.bat](Drone/RUN_DEMO.bat) or use **File → Open World** in Webots and select [worlds/hybrid_wheels.wbt](Drone/worlds/hybrid_wheels.wbt). If a detached copy says **No Project**, open the world from inside `Drone/worlds/`.
4. Click **Reset Simulation** and then **Real-time** if paused. The drone rolls to an inspection point, captures an image, takes off, surveys 17 stops, returns home and lands. Use **View → Console** for controller messages. **Fast** mode can shorten the wait. The local demo runs independently of ROS 2.

| World | Starts with |
| --- | --- |
| [hybrid_wheels.wbt](Drone/worlds/hybrid_wheels.wbt) | Wheel drive, inspection, flight survey |
| [hybrid.wbt](Drone/worlds/hybrid.wbt) | Four-leg walking, inspection, flight survey |
| [farm.wbt](Drone/worlds/farm.wbt) | Flight survey with the default viewpoint |
| `overview.wbt`, `follow.wbt`, `closeup.wbt`, `top.wbt`, `photos.wbt` | Farm survey camera or viewpoint variants |
| `livo2.wbt`, `livo2_fast.wbt`, `hybrid_ros2.wbt`, `hybrid_wheels_ros2.wbt` | Sensor integration variants |

### Controls

Click the 3D view while the simulation is running, then use these keys.

| Key | Action |
| --- | --- |
| `W` / `I` | Walk / take off and resume flight in the hybrid worlds |
| `T` | Roll in the wheel worlds. Take off from a landed state in the flight-only worlds. |
| `M` / `N` / `G` | Cycle mission preset / skip a survey stop / return home |
| `O` / `R` / `V` | Toggle mapped test obstacle / replan / toggle route overlay |
| `D` / `Y` / `Z` | Toggle hybrid sonar handling / inspect terrain / request terrain-checked landing |
| `C` / `B` | Capture downward / front image in the configured image format |
| `K` / `F` | Capture downward / front PNG |
| `A` / `P` | Toggle automatic photos / pause or resume UAV motion |
| `+` / `-` and `H` / `J` | Adjust flight speed and altitude |
| `U` | Load the saved manual route |

Air-to-ground mode requests depend on the site's depth assessment and available ground route. Use `hybrid.wbt` to demonstrate walking directly at startup. Close camera overlays in the 3D view when they obscure the farm.

## Manage missions and outputs

- Edit [settings.json](Drone/controllers/drone/settings.json) for speed, altitude, clearance, photo options and sensor streaming. Reload the world after changing settings. The default survey uses a 0.75 m planning grid, 1.2 m clearance, 3.8 m/s flight speed, 13 m altitude and four-second waypoint holds.
- Open [editor.bat](Drone/editor.bat) for the waypoint editor. Save the route to [manual_mission.json](Drone/controllers/drone/manual_mission.json), then press `U` in Webots. An invalid route is rejected and the active mission is kept. The editor requires Python's Tkinter.
- [scene.json](Drone/controllers/drone/scene.json) defines the mapped field boundaries, home point, obstacles and preset stops. Keep it synchronized with the scene in `worlds/` when editing farm geometry.
- Each run writes to [captures/](Drone/captures/). Open the latest `MISSION_GALLERY.html` or `photo_manifest.csv` for saved frames. The hybrid worlds tag the pre-flight image as `GROUND_INSPECTION`. `hybrid_status.json` and `CONSOLE_TELEMETRY.jsonl` record movement and final landing. Start [sensors.bat](Drone/sensors.bat) to open the live dashboard.
- The front and downward survey cameras produce 3840 × 2160 images. The separate streaming camera has lower resolution. A default hybrid survey with two PNGs per stop uses several hundred MB. Archive old runs when you no longer need them. `capture_front_also` and `photo_format` control future output size.
- The original roof PNG was truncated inside its archive. The working copy uses the bundled 4096 × 4096 farm-wood texture at the roof's asset path. The supplied ZIP manifests describe the original distribution.

## Test and troubleshoot

Run the offline tests from the project root.

```powershell
python -m unittest discover -s Drone/tests -v
```

The Windows [test launcher](Drone/RUN_OFFLINE_TESTS.bat) runs the same suite from `Drone/`. PyYAML is only needed for the ROS calibration-config test and tool. To smoke-test the rendered demo, start a fresh run, watch `ROLLING` or `WALKING` transition to `FLYING` and check for `[MISSION COMPLETE]` and `LANDED`. A default hybrid run writes one ground image plus two images at each of the 17 survey stops. Check the stop IDs and camera names in the manifest against the selected mission.

### Verification on this PC

| Webots run | First logged landing | Survey output |
| --- | --- | --- |
| Wheel drive to flight | 196.05 simulated seconds, 0.091 m from home | 17 stops, 35 decodable 3840 × 2160 PNGs |
| Leg walking to flight | 198.05 simulated seconds, 0.091 m from home | 17 stops, 35 decodable 3840 × 2160 PNGs |

The wheel run confirms the corrected 0.60 m model landing height and the ground inspection label. The walking run was recorded before those two changes. The local TCP monitor received live IMU, LiDAR, RGB and GPS data from Webots and wrote a camera sample and a point cloud. The offline suite passed 65 tests and skipped the PyYAML-dependent calibration test.

| Symptom | Check |
| --- | --- |
| Pink checkerboards | Open the world inside the assembled project and check for the matching files in `Drone/textures/`. |
| Controller stops or simulation stays still | Switch to Real-time, check **View → Console** for the first traceback and confirm Webots is using the intended Python executable. |
| Test obstacle is refused | Try `O` during a longer flight leg so the controller has room to place the obstacle. |
| Air-to-walk or air-to-roll is rejected | Press `Y` to see the depth assessment. The controller requires a safe footprint and a usable ground route for descent. |
| TCP status says disconnected | Start the monitor or ROS 2 receiver before the world. A disconnected receiver does not stop the local mission. |

For source edits, use [controllers/drone/](Drone/controllers/drone/) for flight, planning and sensors, [controllers/hybrid/](Drone/controllers/hybrid/) for walking, and [controllers/wheels/](Drone/controllers/wheels/) for wheels. Use [tools/add_wheels_worlds.py](Drone/tools/add_wheels_worlds.py) after editing the hybrid world sources if the wheel variants need regeneration. Read the [generator guide](Drone/docs/guide.md) before running broader rebuilds. Webots may offer to save controller-generated scene changes when closing a world after a run. Discard those runtime changes during routine testing so the source world opens in its original starting state.

## Optional sensor bridge

Run [integration/monitor.bat](Drone/integration/monitor.bat) and start a compatible world. The TCP monitor listens on `127.0.0.1:15882` by default and reports IMU, LiDAR, RGB and GPS counts. It writes sample files under `integration/recordings/`. For Ubuntu/WSL2 ROS 2 publishing, see the [ROS 2 connection guide](Drone/docs/ROS2_CONNECTION.md). For the external estimator, see the [FAST-LIVO2 setup](Drone/docs/FAST_LIVO2_SETUP.md).

## Report scope

| Design report | Workspace coverage |
| --- | --- |
| §4.3 Webots navigation | Farm, known-map 2D A*, waypoint editing, obstacle replanning, survey images and return-home. Leg and wheel demonstrations extend the navigation sandbox. |
| §4.2 Structure, wing folding and flight/leg dynamics | The MATLAB studies and URDF are separate inputs. The Webots quadcopter models locomotion visually. |
| §4.4–4.7 IoT, crop-health AI, fusion, visitor routing and dashboards | Separate subsystem experiments. Reproducing them requires their source, datasets and model weights. The local Webots dashboard shows sensor telemetry. |
| FAST-LIVO2 and physical KPIs | The TCP bridge supplies simulated measurements. Estimator convergence, power, thrust, contact force and tracking accuracy require separate validation. |
