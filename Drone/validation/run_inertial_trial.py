"""Run isolated Webots/ROS trials using the prepared LIO-SAM test container."""
import argparse,json,os,subprocess,threading,time
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('--container',required=True,help='Dedicated prepared test container; restarted for each clock reset')
p.add_argument('--context',default='kiri')
p.add_argument('--mode',choices=('camera','lidar','combined','calibration'),required=True)
p.add_argument('--scenario',choices=('static','moving','multiple'),default='static')
p.add_argument('--output',type=Path,required=True)
p.add_argument('--world',default='inertial_flight.wbt')
p.add_argument('--mission',type=Path)
p.add_argument('--gravity',default='physical')
p.add_argument('--profile',choices=('arena', 'farm'),default='arena')
p.add_argument('--port',type=int,default=15882)
p.add_argument('--domain-id',type=int,default=12,
               help='Use a different ROS domain for each concurrent validation lane')
p.add_argument('--timeout',type=float,default=150.)
a=p.parse_args()
out=a.output.resolve()
out.mkdir(parents=True,exist_ok=False)
base=Path(__file__).resolve().parent
commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=base, text=True).strip()
(out/'run.json').write_text(json.dumps(dict(commit=commit, mode=a.mode, scenario=a.scenario,
                                           world=a.world, gravity=a.gravity, profile=a.profile,
                                           port=a.port, ros_domain_id=a.domain_id), indent=2))
docker=['docker','--context',a.context]
def run(*args):subprocess.run(docker+list(args),check=True,stdout=subprocess.DEVNULL)
run('restart','-t','1',a.container)
run('cp',str(base.parent/'ros2_ws/src/drone_ros_bridge')+'/.',a.container+':/tmp/bridge-src/drone_ros_bridge')
run('exec',a.container,'bash','-lc','source /opt/ros/humble/setup.bash && cd /tmp/bridge-ws && colcon build --base-paths /tmp/bridge-src --packages-select drone_ros_bridge')
run('cp',str(base/'observe_flight.py'),a.container+':/tmp/observe_flight.py')
source='source /opt/ros/humble/setup.bash && source /lio_sam_ws/install/setup.bash && source /tmp/bridge-ws/install/setup.bash && '
domain = ['-e', 'ROS_DOMAIN_ID=%d' % a.domain_id]
run('exec',*domain,'-d',a.container,'bash','-lc',source+'exec ros2 launch drone_ros_bridge lio_sam.launch.py host:=0.0.0.0 gravity_mode:=%s profile:=%s'%(a.gravity,a.profile)+' >/tmp/physics.log 2>&1')
watch=subprocess.Popen(docker+['exec',*domain,a.container,'bash','-lc',source+'exec python3 -u /tmp/observe_flight.py'],stdout=subprocess.PIPE,text=True)
def relay():
    with (out/'ros.jsonl').open('w') as log:
        for line in watch.stdout:
            print(line.rstrip(),file=log)
            record=json.loads(line)
            if record[0]=='nav':
                temp=out/'pose.tmp';temp.write_text(line)
                for _ in range(8):
                    try:temp.replace(out/'pose.json');break
                    except PermissionError:time.sleep(.001)
worker=threading.Thread(target=relay);worker.start()
try:
    time.sleep(2)
    env=dict(os.environ,UAV_EVIDENCE=str(out),UAV_SCENARIO=a.scenario,UAV_LIO_PORT=str(a.port))
    if a.mode!='calibration':env['UAV_SENSOR_MODE']=a.mode
    if a.mission:env['UAV_MISSION_JSON']=str(a.mission.resolve())
    subprocess.run([os.environ.get('WEBOTS_EXE','C:/Program Files/Webots/msys64/mingw64/bin/webots.exe'),'--batch','--mode=fast',str(Path(a.world) if Path(a.world).is_absolute() else base/'worlds'/a.world)],env=env,check=True,timeout=a.timeout)
    time.sleep(1)
finally:
    run('cp',a.container+':/tmp/physics.log',str(out/'ros.log'))
    run('stop','-t','1',a.container)
    watch.wait(timeout=10);worker.join()
