"""Live ROS topic arrival report. Does not claim estimator success."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Image, PointCloud2


class Health(Node):
    def __init__(self):
        super().__init__('webots_sensor_health')
        self.counts = {'imu': 0, 'lidar': 0, 'image': 0}
        self.create_subscription(Imu, '/livox/imu',
                                 lambda m: self.receive('imu', m), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/webots/lidar/points',
                                 lambda m: self.receive('lidar', m), qos_profile_sensor_data)
        self.create_subscription(Image, '/webots/slam/image_raw',
                                 lambda m: self.receive('image', m), qos_profile_sensor_data)
        self.create_timer(5.0, self.report)

    def receive(self, key, message):
        self.counts[key] += 1

    def report(self):
        self.get_logger().info('last 5 s: IMU=%d  LiDAR=%d  RGB=%d' % (
            self.counts['imu'], self.counts['lidar'], self.counts['image']))
        if any(v == 0 for v in self.counts.values()):
            self.get_logger().warning('Missing stream - inspect Webots/WSL2 connection and enabled devices')
        self.counts = dict.fromkeys(self.counts, 0)


def main():
    rclpy.init()
    node = Health()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
