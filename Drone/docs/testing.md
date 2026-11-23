# Verification

The included `tests/` directory verifies path planning, manual waypoint validation, controller state changes, simulated camera-save calls and landing logic under synthetic sensor input. Run `test.bat` on Windows or `python -m unittest discover -s tests -v` from the project directory.

These tests use mocked Webots objects and generated depth arrays. They do **not** measure Webots GUI frame rate, physical flight dynamics, true SLAM accuracy, calibrated force sensing or a real end-to-end drone survey. Before submitting screenshots or metrics, run the actual `farm.wbt` in Webots and record genuine results.
