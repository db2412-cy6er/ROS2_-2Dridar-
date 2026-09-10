from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xuegecar_yolo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chuyun',
    maintainer_email='16093243+czulcl@user.noreply.gitee.com',
    description='YOLO detection node (yolo26n) for the Leap physical car semantic navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = xuegecar_yolo.yolo_detector:main',
        ],
    },
)
