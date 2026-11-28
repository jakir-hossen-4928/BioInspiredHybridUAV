"""Run the two existing Webots demonstrations and own their local processes."""
import argparse
import json
import os
from datetime import datetime
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
import webbrowser

from verify_live import kill_tree

DRONE = Path(__file__).resolve().parents[1]
SESSIONS = DRONE.parent / 'client-demo-sessions'
ACTIVE = SESSIONS / 'active.json'
READY = ('Listening for Webots', 'IMU Preintegration Started',
         'Image Projection Started', 'Feature Extraction Started', 'Map Optimization Started')


def atomic_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2), encoding='utf-8')
    try:
        temporary.replace(path)
    except PermissionError:
        # Webots can briefly have the last pose open on Windows.
        pass


def relay(observer, folder):
    counts = dict(scan=0, odom=0, nav=0, map=0, imu=0, camera=0)
    last_print = -5.
    latest_pose = None
    map_points = 0
    with (folder / 'ros.jsonl').open('w', encoding='utf-8', buffering=1) as log:
        for line in observer.stdout:
            log.write(line)
            try:
                kind, stamp, value = json.loads(line)
            except (ValueError, TypeError):
                continue
            if kind in counts:
                counts[kind] += 1
            if kind == 'nav':
                atomic_json(folder / 'pose.json', [kind, stamp, value])
                latest_pose = value[:3]
            elif kind == 'map':
                map_points = len(value)
            if stamp - last_print >= 5:
                atomic_json(folder / 'ros-live.json', dict(samples=counts, lio_xyz=latest_pose,
                                                           map_points=map_points, stamp_s=stamp))
                print(f'[{folder.name} {stamp:.1f}s] {counts} LIO-SAM XYZ {latest_pose}', flush=True)
                last_print = stamp


def demo(args, session):
    context = os.environ.get('UAV_CLIENT_CONTEXT', 'kiri')
    container = os.environ.get('UAV_CLIENT_CONTAINER', 'uav-liosam-nov27')
    port = int(os.environ.get('UAV_CLIENT_PORT', '15882'))
    domain = os.environ.get('UAV_CLIENT_ROS_DOMAIN', '62')
    docker = ['docker', '--context', context]
    ros_env = ['-e', f'ROS_DOMAIN_ID={domain}']
    source = ('source /opt/ros/humble/setup.bash && source /lio_sam_ws/install/setup.bash && '
              'source /tmp/bridge-ws/install/setup.bash && ')
    cancelled = session / 'stop.request'

    def docker_run(*command, **kwargs):
        return subprocess.run(docker + list(command), check=True, text=True, timeout=kwargs.pop('timeout', 25),
                              **kwargs)

    def check_stop():
        if cancelled.exists():
            raise InterruptedError('Stop requested.')

    webots = os.environ.get('WEBOTS_EXE') or shutil.which('webots') or (
        r'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe')
    if not Path(webots).is_file():
        raise RuntimeError('Webots R2025a not found. Set WEBOTS_EXE to webots.exe.')
    version = subprocess.run([webots, '--version'], check=True, text=True, capture_output=True, timeout=15)
    if 'R2025a' not in version.stdout + version.stderr:
        raise RuntimeError('The client demonstration requires Webots R2025a.')
    if not shutil.which('docker'):
        raise RuntimeError('Docker CLI was not found.')
    docker_run('info', '--format', '{{.ServerVersion}}')
    running = docker_run('inspect', '--format', '{{.State.Running}}', container,
                         capture_output=True).stdout.strip()
    if running != 'false':
        raise RuntimeError(f'{container} is already running. Let its current job finish before starting the demo.')
    mapping = docker_run('inspect', '--format', '{{json .HostConfig.PortBindings}}', container,
                         capture_output=True).stdout
    if not any(entry['HostPort'] == str(port) for entry in (json.loads(mapping) or {}).get('15882/tcp', [])):
        raise RuntimeError(f'{container} must publish container port 15882 on host port {port}.')
    prepared = False
    status = {}
    for name, world, gravity in (
            ('farm', DRONE / 'worlds/hybrid_wheels_ros2.wbt', 'zero-world'),
            ('lio-sam', DRONE / 'validation/worlds/inertial_flight.wbt', 'physical')):
        folder = session / name
        folder.mkdir()
        observer = webots_process = rviz = None
        worker = None
        owned_container = False
        try:
            check_stop()
            print(f'[{name}] Starting {container}. Evidence: {folder}', flush=True)
            docker_run('start', container)
            owned_container = True
            if not prepared:
                docker_run('exec', container, 'mkdir', '-p', '/tmp/bridge-ws',
                           '/tmp/bridge-src/drone_ros_bridge')
                docker_run('cp', str(DRONE / 'ros2_ws/src/drone_ros_bridge') + '/.',
                           container + ':/tmp/bridge-src/drone_ros_bridge')
                with (session / 'bridge-build.log').open('w', encoding='utf-8') as log:
                    docker_run('exec', container, 'bash', '-lc',
                               'source /opt/ros/humble/setup.bash && test -f /lio_sam_ws/install/setup.bash && '
                               'cd /tmp/bridge-ws && colcon build --base-paths /tmp/bridge-src '
                               '--packages-select drone_ros_bridge', stdout=log, stderr=subprocess.STDOUT,
                               timeout=180)
                docker_run('cp', str(DRONE / 'validation/observe_flight.py'), container + ':/tmp/client-observe.py')
                prepared = True
            docker_run('exec', *ros_env, '-d', container, 'bash', '-lc', source +
                       f'exec ros2 launch drone_ros_bridge lio_sam.launch.py host:=0.0.0.0 '
                       f'gravity_mode:={gravity} >/tmp/client-demo-ros.log 2>&1')
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                check_stop()
                log = subprocess.run(docker + ['exec', container, 'cat', '/tmp/client-demo-ros.log'],
                                     text=True, capture_output=True, timeout=10).stdout
                if all(marker.lower() in log.lower() for marker in READY):
                    break
                time.sleep(.5)
            else:
                raise RuntimeError('ROS receiver or LIO-SAM nodes did not announce readiness. See ros.log.')
            observer = subprocess.Popen(docker + ['exec', *ros_env, '-e', 'UAV_CLIENT_DEMO=1',
                                        container, 'bash', '-lc', source + 'exec python3 -u /tmp/client-observe.py'],
                                        stdout=subprocess.PIPE, encoding='utf-8')
            worker = threading.Thread(target=relay, args=(observer, folder), daemon=True)
            worker.start()
            display = os.environ.get('UAV_RVIZ_DISPLAY') or os.environ.get('DISPLAY')
            if display:
                probe = subprocess.run(docker + ['exec', container, 'test', '-x', '/opt/ros/humble/bin/rviz2'],
                                       capture_output=True, timeout=10)
                if probe.returncode == 0:
                    docker_run('cp', str(DRONE / 'validation/client_live.rviz'), container + ':/tmp/client-live.rviz')
                    rviz = subprocess.Popen(docker + ['exec', *ros_env, '-e', f'DISPLAY={display}',
                                           container, 'bash', '-lc', 'source /opt/ros/humble/setup.bash && '
                                           'exec rviz2 -d /tmp/client-live.rviz'])
                else:
                    print('RViz skipped: rviz2 is unavailable in the demo container.')
            else:
                print('RViz skipped: no graphical Linux DISPLAY configured. ROS data are logged live.')
            env = dict(os.environ, UAV_LIO_PORT=str(port), PYTHONUNBUFFERED='1',
                       UAV_CLIENT_STOP=str(cancelled))
            for key in ('UAV_CLIENT_EVIDENCE', 'UAV_SENSOR_MODE', 'UAV_MISSION_JSON', 'UAV_SCENARIO'):
                env.pop(key, None)
            if name == 'farm':
                env['UAV_CLIENT_EVIDENCE'] = str(folder)
            else:
                env.update(UAV_EVIDENCE=str(folder), UAV_SENSOR_MODE='combined', UAV_SCENARIO='moving',
                           UAV_CLIENT_CAMERA='1')
            with (folder / 'webots.log').open('w', encoding='utf-8') as log:
                webots_process = subprocess.Popen([webots, '--batch', '--stdout', '--stderr',
                                                   '--mode=' + args.mode, str(world)],
                                                  env=env, stdout=log, stderr=subprocess.STDOUT)
                deadline = time.monotonic() + (900 if name == 'farm' else 150)
                dashboard_opened = False
                while webots_process.poll() is None:
                    if cancelled.exists():
                        try:
                            webots_process.wait(timeout=8)
                        except subprocess.TimeoutExpired:
                            pass
                        check_stop()
                    if time.monotonic() >= deadline:
                        raise RuntimeError(f'{name} Webots run timed out.')
                    dashboard = folder / 'dashboard.html'
                    if name == 'farm' and dashboard.is_file() and not dashboard_opened:
                        dashboard_opened = True
                        print(f'Opening live dashboard {dashboard}', flush=True)
                        if not args.no_dashboard:
                            webbrowser.open(dashboard.as_uri())
                    time.sleep(.25)
            result = folder / ('farm-result.json' if name == 'farm' else 'result.json')
            check_stop()
            if webots_process.returncode or not result.is_file():
                raise RuntimeError(f'Webots exited without a {name} result. See webots.log.')
            data = json.loads(result.read_text(encoding='utf-8'))
            status[name] = ({'state': data['state'], 'photos': data['photos'],
                             'wheel_distance_m': data['wheel_distance_m'],
                             'walking_distance_m': data['walking_distance_m'],
                             'returned_home': data['returned_home']}
                            if name == 'farm' else {key: data[key] for key in
                                                   ('passed', 'reason', 'contact_steps', 'goal_error_m')})
        except InterruptedError:
            status[name] = {'state': 'STOPPED'}
            raise
        except (OSError, RuntimeError, subprocess.SubprocessError, ValueError) as exc:
            status[name] = {'error': str(exc)}
            print(f'[{name}] {exc}', file=sys.stderr)
        finally:
            kill_tree(webots_process)
            kill_tree(rviz)
            if owned_container:
                try:
                    subprocess.run(docker + ['cp', container + ':/tmp/client-demo-ros.log',
                                             str(folder / 'ros.log')], capture_output=True, timeout=15)
                finally:
                    docker_run('stop', '-t', '2', container)
            kill_tree(observer)
            if worker is not None:
                worker.join(timeout=5)
            atomic_json(session / 'session-result.json', status)
    farm = status.get('farm', {})
    return 0 if (farm.get('state') == 'LANDED' and farm['returned_home']
                 and min(farm['photos'], farm['wheel_distance_m'], farm['walking_distance_m']) > 0
                 and status.get('lio-sam', {}).get('passed')) else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stop', action='store_true')
    parser.add_argument('--mode', choices=('realtime', 'fast'), default='realtime')
    parser.add_argument('--no-dashboard', action='store_true', help='Save the dashboard without opening a browser')
    args = parser.parse_args()
    if args.stop:
        try:
            owner = json.loads(ACTIVE.read_text(encoding='utf-8'))
        except FileNotFoundError:
            print('No client demonstration is active.')
            return 0
        (SESSIONS / owner['session'] / 'stop.request').touch()
        print('Stop requested. The launcher will close its Webots and ROS processes.')
        return 0
    if sys.version_info < (3, 9):
        raise SystemExit('Python 3.9 or later is required.')
    SESSIONS.mkdir(exist_ok=True)
    session = SESSIONS / datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    try:
        with ACTIVE.open('x', encoding='utf-8') as lock:
            json.dump(dict(session=session.name, pid=os.getpid()), lock)
    except FileExistsError:
        raise SystemExit(f'Another demo owns {ACTIVE}. Use STOP_CLIENT_DEMO.bat.')
    try:
        session.mkdir()
        print(f'Client demo evidence: {session}', flush=True)
        return demo(args, session)
    except (InterruptedError, KeyboardInterrupt):
        print('Client demo stopped.')
        return 0
    except (OSError, RuntimeError, subprocess.SubprocessError, ValueError) as exc:
        atomic_json(session / 'session-result.json', {'startup_error': str(exc)})
        print(exc, file=sys.stderr)
        return 1
    finally:
        ACTIVE.unlink(missing_ok=True)


if __name__ == '__main__':
    sys.exit(main())
