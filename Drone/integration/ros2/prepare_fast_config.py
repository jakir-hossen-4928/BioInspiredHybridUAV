#!/usr/bin/env python3
"""Derive FAST-LIVO2 Webots configs from the *installed upstream fork* YAML.

This is not camera/IMU/LiDAR calibration. Its extrinsics are initial placeholders
and must be checked before treating trajectory/map data as accurate.
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
    p['preprocess']['lidar_type'] = 8   # only the *message convention*, not a Mid-360 scan emulator
    p['time_offset'].update(imu_time_offset=0.0, img_time_offset=0.0)
    p['rolling_shutter']['enable'] = False
    p['rolling_shutter']['td_estimate_enable'] = False
    # Extrinsics below are deliberately uncalibrated identity / zero placeholders.
    # Do not silently inherit calibration from a different physical device.
    p['extrin_calib'].update(
        extrinsic_T=[0.0, 0.0, 0.0],
        extrinsic_R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        Rcl=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        Pcl=[0.0, 0.0, 0.0])
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
        target.write_text('# UNCALIBRATED SIMULATION INITIAL VALUES - VERIFY EXTRINSICS\n' +
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
    print('Set actual IMU/LiDAR/camera extrinsics before evaluating localization accuracy')

if __name__ == '__main__':
    main()
