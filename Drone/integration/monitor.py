"""Simple sensor-stream receiver for Windows; ROS 2 is not required.

This only checks that live Webots data are delivered. It is NOT FAST-LIVO2.
Writes one point-cloud PLY and one RGB BMP frame to integration/recordings.
"""
import argparse
import json
import socket
import struct
import time
import zlib
from pathlib import Path
from ros2.livo_protocol import recv_frame, unpack_points


def save_ply(path, records):
    with path.open('w', encoding='utf-8') as file:
        file.write('ply\nformat ascii 1.0\n')
        file.write('element vertex %d\n' % len(records))
        file.write('property float x\nproperty float y\nproperty float z\nend_header\n')
        for x,y,z,*_ in records:
            file.write('%.5f %.5f %.5f\n' % (x,y,z))


def save_bmp(path, bgra, width, height):
    # BMP rows are bottom-up and 24-bit BGR; each row is padded to 4 bytes.
    row_size = (width*3 + 3) & ~3
    pixel_size = row_size*height
    with path.open('wb') as file:
        file.write(b'BM' + struct.pack('<IHHI', 54+pixel_size,0,0,54))
        file.write(struct.pack('<IiiHHIIiiII',40,width,height,1,24,0,pixel_size,2835,2835,0,0))
        padding = b'\x00' * (row_size-width*3)
        for row in range(height-1, -1, -1):
            start=row*width*4
            for i in range(width):
                file.write(bgra[start+4*i:start+4*i+3])
            file.write(padding)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=15882)
    options=parser.parse_args()
    folder=Path(__file__).resolve().parent/'recordings'
    folder.mkdir(parents=True,exist_ok=True)
    count={'imu':0,'lidar':0,'image':0,'gps':0}
    first={'lidar':False,'image':False}
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        server.bind((options.host,options.port))
        server.listen(1)
        print('Waiting for Webots sensor stream at %s:%d' % (options.host,options.port))
        print('Open Webots worlds/livo2.wbt and press Play. Ctrl+C to stop.')
        while True:
            conn,address=server.accept()
            with conn:
                print('Connected:',address)
                while True:
                    try:
                        meta,payload=recv_frame(conn)
                    except (OSError,EOFError):
                        print('Disconnected. Waiting for Webots again.')
                        break
                    typ=meta['type']
                    if typ in count:count[typ]+=1
                    if typ=='lidar':
                        pts=unpack_points(payload)
                        if not first['lidar']:
                            save_ply(folder/'lidar_sample.ply',pts)
                            print('Saved LiDAR point cloud:',len(pts),'points')
                            first['lidar']=True
                    elif typ=='image' and not first['image']:
                        raw=zlib.decompress(payload)
                        save_bmp(folder/'camera_sample.bmp',raw,int(meta['width']),int(meta['height']))
                        print('Saved camera sample:',meta['width'],'x',meta['height'])
                        first['image']=True
                    if typ=='imu' and count['imu']%20==0:
                        print('Live frames',count,'sim_time=%.2f'%meta['sim_time_s'])


if __name__=='__main__':main()
