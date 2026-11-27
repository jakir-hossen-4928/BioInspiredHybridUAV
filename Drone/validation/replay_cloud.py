"""Publish the last recorded global map in ROS 2 for RViz inspection."""
import json
import sys
import rclpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header

cloud = None
with open(sys.argv[1]) as f:
    for line in f:
        kind,t,value=json.loads(line)
        if kind=='map' and value and isinstance(value[0],list): cloud=value
if not cloud: raise ValueError('No recorded global cloud')
rclpy.init()
n=rclpy.create_node('evidence_replay')
pub=n.create_publisher(PointCloud2,'/evidence/map',1)
msg=create_cloud_xyz32(Header(frame_id='odom'),cloud)
n.create_timer(.5,lambda: pub.publish(msg))
rclpy.spin(n)
