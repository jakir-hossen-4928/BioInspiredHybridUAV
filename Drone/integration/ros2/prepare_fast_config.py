#!/usr/bin/env python3
"""Derive FAST-LIVO2 Webots configs from the *installed upstream fork* YAML.

The extrinsics come from the two hybrid ROS 2 worlds' nominal sensor mounts.
Revalidate them if the world geometry changes or you assess estimation accuracy.
"""
import argparse
import math
from pathlib import Path
import yaml


def update_configs(mid360, camera, out_dir, *, width=640, height=480, hfov=1.05):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    mid = yaml.safe_load(Path(mid360).read_text())
    cam = yaml.safe_load(Path(camera).read_text())
    for name, data in [('mid360', mid), ('camera', cam)]:
        if not isinstance(data, dict) or '/**' not in data or 'ros__parameters' not in data['/**']:
            raise ValueError('%s config does not match the selected FAST-LIVO2 ROS2 fork' % name)
    p = mid['/**']['ros__parameters']
    required = ('common','extrin_calib','preprocess','time_offset','rolling_shutter')
    missing = [k for k in required if k not in p]
    if missing:
        raise ValueError('upstream config changed, missing sections '+repr(missing))
    p['use_sim_time'] = True
    p['common'].update(img_topic='/webots/slam/image_raw',
                       lid_topic='/livox/lidar', imu_topic='/livox/imu',
                       img_en=1, lidar_en=1, livox_msg_type='livox_ros_driver2')
    # The U-AMC subscriber selects CustomMsg only in the AVIA (1) branch.
    p['preprocess']['lidar_type'] = 1
    p['preprocess']['scan_line'] = 24
    p['time_offset'].update(imu_time_offset=0.0, img_time_offset=0.0)
    p['rolling_shutter']['enable'] = False
    p['rolling_shutter']['td_estimate_enable'] = False
    # LiDAR is 0.555 m above the IMU. Camera optical axes are right/down/forward.
    # Pcl is Rcl times (LiDAR origin minus camera origin), in body coordinates.
    p['extrin_calib'].update(
        extrinsic_T=[0.0, 0.0, 0.555],
        extrinsic_R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        Rcl=[0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0],
        Pcl=[0.0, -0.82065, -0.69])
    cp = cam['/**']['ros__parameters']
    cp['use_sim_time'] = True
    fx = width / (2.0 * math.tan(hfov/2.0))
    cp.update(cam_model='Pinhole', cam_width=width, cam_height=height,
              scale=1.0, cam_fx=fx, cam_fy=fx,
              cam_cx=(width-1)/2, cam_cy=(height-1)/2,
              cam_d0=0.0, cam_d1=0.0, cam_d2=0.0, cam_d3=0.0)
    mid_path = out / 'mid360_webots.yaml'
    cam_path = out / 'camera_webots.yaml'
    for target, data in [(mid_path,mid),(cam_path,cam)]:
        target.write_text('# NOMINAL HYBRID ROS 2 WEBOTS SENSOR GEOMETRY\n' +
                          yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    return mid_path, cam_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--upstream', default=str(Path.home()/'fast_ws/src/fast_livo'))
    ap.add_argument('--output', default=str(Path.home()/'drone_ros_config'))
    args=ap.parse_args()
    root=Path(args.upstream)
    files=update_configs(root/'config/mid360_lvi.yaml',
                         root/'config/camera_pinhole.yaml', args.output)
    print('Created configs:', *(str(f) for f in files))
    print('Verify the nominal extrinsics and timing before evaluating localization accuracy')

if __name__ == '__main__':
    main()
