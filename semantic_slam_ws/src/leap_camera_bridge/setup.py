from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'leap_camera_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chuyun',
    maintainer_email='chuyun@example.com',
    description='HTTP MJPEG -> ROS2 Image bridge with P0 time-sync X-Capture-* headers.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_http_bridge = leap_camera_bridge.camera_http_bridge:main',
            'check_p0_time = leap_camera_bridge.check_p0_time:main',
        ],
    },
)
