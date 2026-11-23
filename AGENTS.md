# AGENTS.md

## Shape of the repo

- Single Webots project. **Everything lives under `Drone/`** — the repo root holds only `README.md`, `.gitignore` and this file. There is no `src/`, no package manifest, no `requirements.txt`.
- Controller import direction (each does `sys.path.insert(...)` on its sibling): `controllers/drone/` is the base library ← `controllers/hybrid/` ← `controllers/wheels/`. Touching `drone/` changes walking and rolling too.
- `Drone/worlds/*.wbt` are partly **generated** from `Drone/tools/*.py` (`create_worlds.py`, `add_wheels_worlds.py`). Don't hand-edit a world and then run `Drone/rebuild.bat`: it overwrites the worlds (backing them up to `world_backups/` first). Regenerate through the tools, or edit only worlds the generator doesn't own.

## Tests — the entire quality gate

There is **no lint, typecheck, formatter, codegen or CI config** anywhere in the repo. The unittest suite is all there is.

```powershell
# from repo root
python -m unittest discover -s Drone/tests -v                      # full suite, ~5 s
python -m unittest discover -s Drone/tests -p "test_navigation.py" -v   # one module
```

- Don't use a dotted path like `Drone.tests.test_navigation`: `tests/` has no `__init__.py`, and `test_routes.py` does `from test_navigation import FakeSupervisor`, so discovery must put `tests/` on `sys.path`.
- From inside `Drone/` use `-s tests` instead — that's what `RUN_OFFLINE_TESTS.bat` and `test.bat` do (they prefer the `py -3` launcher when present).
- **The `Ran N tests` / `OK` / `FAILED` summary goes to stderr, while stdout gets ~750 lines of controller banner text.** Merged/tail output ends with the banner, not the result — redirect the streams separately or grep for `Ran `. Trust the exit code.
- Baseline: **66 tests, `OK (skipped=1)`**. The skip is `test_ros2_package.test_prepare_config_replaces_external_calibration`, which only runs if PyYAML is installed.
- Webots is **not** required — controllers are mocked, all file writes go to `tempfile` dirs. Exception: `test_livo_stream.py` opens a real TCP socket on `127.0.0.1` with 4 s socket timeouts and a 2 s thread join, so a loaded machine can fail it. Re-run before calling it a regression.

## Git / GitHub

- Remote: private `jakir-hossen-4928/BioInspiredHybridUAV`, branch `main`, HTTPS authenticated via `gh auth setup-git` + Windows Credential Manager.
- `.gitignore` blocks `/LaTeX/` and `*.pdf`. Three PDFs exist on disk (root `Bio-Inspired Hybrid UAV.pdf`, `Drone/docs/report.pdf`, `Drone/docs/Technical_Progress_Report_21_Nov_2026.pdf`) and must stay untracked — never `git add -f` them. Same for `__pycache__/` and `*.pyc`.
- **The worktree is dirty by design while a Webots session runs.** `Drone/captures/<session>/` and `Drone/dashboard.html` are tracked live outputs being rewritten in place, so `git status` never settles. Don't chase it; just avoid sweeping unrelated junk into a commit (`git add -A` will grab the churn).
- Repo is ~1.24 GB, almost entirely 4K capture PNGs (~20 MB each). GitHub's hard per-file limit is 100 MB — don't casually add more large binaries.
- `core.autocrlf=true` with no `.gitattributes`, so "LF will be replaced by CRLF" warnings are expected noise, not a problem to fix.
- `Drone/CHECKSUMS_SHA256.txt` and `Drone/FINAL_FILE_MANIFEST_SHA256.csv` are leftovers from the original ZIP distribution. **No script or test verifies them** — editing source does not mean regenerating them.

## Conventions that differ from the defaults

- **Docs must not overclaim.** The repo's own docs police this: locomotion is a Supervisor-kinematic *visual* demonstration (no wheel traction model, no validated leg force), mapping is a ground-height grid and not SLAM, FAST-LIVO2 is interface-only, and offline tests prove nothing about Webots GUI behaviour. `Drone/docs/screenshots/README.md` explicitly forbids presenting older screenshots as proof of the final worlds. Keep that framing in anything you write; `Drone/docs/specifications.md` is the per-sensor source of truth.
- Configuration is JSON beside the controller, not environment variables: `controllers/drone/settings.json` (speed, altitude, clearance, photos), `scene.json` (boundaries, obstacles, stops), `manual_mission.json` (editor-saved routes). **Reload the world after editing any of them.**
- Optional deps only: PyYAML (ROS calibration test/tool) and Tkinter (route editor GUI). Everything else is stdlib.

## Running the real simulation

- Webots R2025a + Python 3 on Windows. If Webots can't start a controller, set **Tools → Preferences → General → Python command** to the installed interpreter and reopen the world.
- Entrypoint: `Drone/RUN_DEMO.bat`, or open `Drone/worlds/hybrid_wheels.wbt` in Webots, then **Reset** → **Real-time**.
- The TCP sensor bridge listens on `127.0.0.1:15882` (`Drone/integration/monitor.bat`) and must be started **before** the world; a missing receiver never stops the local mission.
