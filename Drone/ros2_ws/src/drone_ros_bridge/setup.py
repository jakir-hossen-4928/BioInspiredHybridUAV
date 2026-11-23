from setuptools import setup
from glob import glob
import os

package_name = 'drone_ros_bridge'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Drone project team',
    maintainer_email='maintainer@example.com',
    description='Webots sensor bridge for hybrid drone simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'webots_receiver = drone_ros_bridge.receiver:main',
            'sensor_health = drone_ros_bridge.health:main',
        ],
    },
)
