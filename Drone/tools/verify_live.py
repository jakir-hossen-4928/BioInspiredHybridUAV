"""Run one live Webots mission with the sensor bridge attached and report PASS/FAIL.

Replaces GUI automation: no computer-use helper, no screenshot reading, no
save-on-quit dialog. Usage:

    python Drone/tools/verify_live.py                    # hybrid_wheels.wbt (wheels demo)
    python Drone/tools/verify_live.py --world hybrid.wbt # walking -> flight -> landing

Exit code 0 only when every required check passes. Evidence (monitor log,
Webots log, capture folder) stays on disk for the report.
"""
import argparse
import glob
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DRONE = HERE.parent
ROOT = DRONE.parent
WEBOTS_CANDIDATES = [
    os.environ.get('WEBOTS_EXE', ''),
    r'C:\Program Files\Webots\msys64\mingw64\bin\webotsw.exe',
    '/usr/local/bin/webots',
]


def find_webots():
    for path in WEBOTS_CANDIDATES:
        if path and Path(path).exists():
            return path
    raise SystemExit('Webots not found; set WEBOTS_EXE to the launcher binary.')


def kill_tree(proc):
    if proc is None or proc.poll() is not None:
        return
    if os.name == 'nt':
        subprocess.run(['taskkill', '/F', '/T', '/PID', str(proc.pid)],
                       capture_output=True, check=False)
    else:
        proc.kill()
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        pass


def newest_capture(before):
    current = set(glob.glob(str(DRONE / 'captures' / '*')))
    fresh = sorted(current - before)
    return Path(fresh[-1]) if fresh else None


def read_rows(folder):
    path = folder / 'CONSOLE_TELEMETRY.jsonl'
    if not path.exists():
        return []
    rows = []
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', default='hybrid_wheels.wbt')
    parser.add_argument('--port', type=int, default=15882)
    parser.add_argument('--timeout', type=float, default=900.0)
    parser.add_argument('--mode', default='fast')
    parser.add_argument('--evidence', default=str(Path(tempfile.gettempdir()) / 'uav-live-evidence'))
    options = parser.parse_args()

    world = DRONE / 'worlds' / options.world
    if not world.exists():
        raise SystemExit('Missing world: %s' % world)
    evidence = Path(options.evidence)
    evidence.mkdir(parents=True, exist_ok=True)
    webots_exe = find_webots()

    before = set(glob.glob(str(DRONE / 'captures' / '*')))
    monitor = webots = None
    checks = []
    try:
        monitor = subprocess.Popen(
            [sys.executable, '-u', 'monitor.py', '--port', str(options.port)],
            cwd=str(DRONE / 'integration'),
            stdout=(evidence / 'monitor.log').open('w', encoding='utf-8'),
            stderr=subprocess.STDOUT)
        time.sleep(3)
        webots = subprocess.Popen(
            [webots_exe, '--mode=%s' % options.mode, str(world)],
            cwd=str(DRONE),
            stdout=(evidence / 'webots.log').open('w', encoding='utf-8'),
            stderr=subprocess.STDOUT)

        folder = None
        deadline = time.time() + options.timeout
        states, rows = [], []
        while time.time() < deadline:
            time.sleep(5)
            if folder is None:
                folder = newest_capture(before)
                if folder is None:
                    continue
            rows = read_rows(folder)
            states = sorted({row.get('state') for row in rows if row.get('state')})
            if 'LANDED' in states and ({'FLYING', 'HOVER'} & set(states)):
                break
            if webots.poll() is not None:
                break

        if folder is None:
            checks.append(('capture session created', False, 'no Drone/captures/ folder appeared'))
        else:
            photos = list(folder.glob('*.png'))
            manifest = folder / 'photo_manifest.csv'
            checks.append(('capture session created', True, folder.name))
            checks.append(('survey photos >= 30', len(photos) >= 30, '%d PNG' % len(photos)))
            checks.append(('photo manifest written', manifest.exists(),
                           manifest.name if manifest.exists() else 'missing'))
            # Walking only happens in the hybrid world; the wheels world rolls.
            if options.world.startswith('hybrid.'):
                checks.append(('walk phase ran', 'WALKING' in states, 'states: %s' % ', '.join(states)))
            checks.append(('flight phase ran', bool({'FLYING', 'HOVER'} & set(states)),
                           'states: %s' % ', '.join(states)))
            checks.append(('mission reached LANDED', 'LANDED' in states, 'states: %s' % ', '.join(states)))
            last = rows[-1] if rows else {}
            sent = last.get('ros_tcp_packets_sent') or 0
            checks.append(('sensor bridge connected', bool(last.get('ros_tcp_connected')),
                           'connected=%s' % last.get('ros_tcp_connected')))
            checks.append(('bridge packets sent > 1000', sent > 1000, '%s packets, %s dropped'
                           % (sent, last.get('ros_tcp_packets_dropped'))))
            checks.append(('planner route nodes > 0', (last.get('planner_route_nodes') or 0) > 0,
                           'route=%s flight_astar=%s' % (last.get('planner_route_nodes'),
                                                         last.get('flight_astar_route_nodes'))))
            checks.append(('height map written', (folder / 'OBSERVED_HEIGHT_MAP.csv').exists(),
                           'OBSERVED_HEIGHT_MAP.csv'))
        monitor_log = (evidence / 'monitor.log').read_text(encoding='utf-8', errors='replace')
        checks.append(('monitor saved PLY + camera sample', 'Saved LiDAR point cloud' in monitor_log
                       and 'Saved camera sample' in monitor_log,
                       'integration/recordings, 640x480 + point cloud'))
    finally:
        kill_tree(webots)
        kill_tree(monitor)
        log = evidence / 'webots.log'
        if log.exists():
            shutil.copy(log, evidence / ('%s.log' % world.stem))

    print()
    for name, ok, detail in checks:
        print('[%s] %-32s %s' % ('PASS' if ok else 'FAIL', name, detail))
    failed = [name for name, ok, _ in checks if not ok]
    print('\n%d checks, %d failed. Evidence: %s' % (len(checks), len(failed), evidence))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
