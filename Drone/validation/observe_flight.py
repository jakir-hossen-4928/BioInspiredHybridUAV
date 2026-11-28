"""ROS-side observer. Only LIO-SAM odometry is sent back to navigation."""
import json
import os
import struct
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as qos
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Imu, Image
from geometry_msgs.msg import PoseStamped

rclpy.init()
node = Node('flight_evidence')
def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
def log(kind, msg, value):
    print(json.dumps([kind, stamp(msg), value]), flush=True)
def pose(p):
    return [p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
def cloud(m):
    fields = {f.name: f.offset for f in m.fields}
    xyz = [[struct.unpack_from('<f', m.data, i+fields[k])[0] for k in 'xyz']
           for i in range(0, len(m.data), m.point_step)]
    log('map', m, xyz)
def scan(m):
    fields = {f.name: f.offset for f in m.fields}
    rings = set()
    offsets = set()
    for i in range(0,len(m.data),m.point_step):
        rings.add(struct.unpack_from('<H',m.data,i+fields['ring'])[0])
        offsets.add(struct.unpack_from('<f',m.data,i+fields['time'])[0])
    log('scan',m,[m.width,sorted(rings),min(offsets),max(offsets)])
node.create_subscription(Odometry,'/lio_sam/mapping/odometry',lambda m:log('odom',m,pose(m.pose.pose)),qos)
node.create_subscription(PoseStamped,'/webots/ground_truth',lambda m:log('truth',m,pose(m.pose)),qos)
node.create_subscription(PointCloud2,'/lio_sam/mapping/map_global',cloud,qos)
node.create_subscription(PointCloud2,'/webots/lidar/points',scan,qos)
if os.environ.get('UAV_CLIENT_DEMO'):
    def preview(kind, extract):
        last = -1.
        def observe(message):
            nonlocal last
            t = stamp(message)
            if t - last >= 1.:
                log(kind, message, extract(message))
                last = t
        return observe
    node.create_subscription(Imu, '/livox/imu', preview('imu', lambda m: [
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]), qos)
    node.create_subscription(Image, '/webots/slam/image_raw', preview('camera', lambda m: [
        m.width, m.height, m.encoding]), qos)
last_nav = -1.
def fused(m):
    global last_nav
    t = stamp(m)
    if t-last_nav >= .04:
        last_nav=t
        v=m.twist.twist.linear
        log('nav',m,pose(m.pose.pose)+[v.x,v.y,v.z])
node.create_subscription(Odometry,'/lio_sam/odometry/imu',fused,qos)
rclpy.spin(node)
