# Setup and editing

**Requirements**: Windows 11, Webots R2025a and Python 3 (including tkinter for the route editor). No ROS installation is needed for the current configuration.

### Running

- `start.bat`: launch the main farm scene, then click **Play** in Webots.
- `editor.bat`: add, drag or delete waypoints on the map, then save the route and press `U` in Webots.
- `sensors.bat`: open the live browser sensor view, refreshed while the simulation runs.
- `photos.bat`: open a high-resolution camera world (greater GPU demand).
- `test.bat`: run offline Python logic tests.
- `rebuild.bat`: regenerate the six world files from the geometry scripts; current files are copied into `world_backups/` first. This will overwrite edits to the worlds. Only run it intentionally.

### Editing parameters

`controllers/drone/settings.json`: speed, altitude, image quality, hover duration, landing slope/roughness thresholds and obstacle clearance.

`controllers/drone/scene.json`: survey stops, mapped obstacles, route boundaries and mission presets.

`controllers/drone/manual_mission.json`: saved custom stops written by the map editor.

`controllers/drone/drone.py`: state machine, camera capture, logging, sensor readouts, visible rotor animation.

`controllers/drone/planner.py`: occupancy-grid A* search.

`controllers/drone/terrain_perception.py`: depth-based local ground plane fit and decision thresholds.

### Data and recordings

On a run, output data is stored under `captures/<session>/`. It may include camera JPGs, a photo manifest, telemetry CSV, a height-map CSV and an HTML photo gallery. The root `dashboard.html` is periodically refreshed from sensor readings.

### Important technical limits

The simulated LiDAR is a generic Webots sensor and does not reproduce Livox Mid-360 measurement timing. GPS uses known simulator coordinates; the method is not FAST-LIVO2 and the route planner is not Hybrid A*. Contact bumpers report touch rather than calibrated force. The full GUI run must be tested on the receiving laptop before results are submitted.


### Full resolution camera capture

The front and downward RGB cameras are 3840 x 2160 in each supplied world.
Set `photo_format` to `png` in `controllers/drone/settings.json` for lossless
photographs; this is the default here. Terrain depth and LiDAR resolutions
are unchanged, so the upgrade is camera and material texture resolution only.
The 4096 x 4096 material textures are resampled from the preserved
1024/2048 pixel originals; they are not native 4K scans.
