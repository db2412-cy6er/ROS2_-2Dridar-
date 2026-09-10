import os

from glob import glob

from setuptools import find_packages, setup


package_name = 'xuegecar_dynamic_semantic_mapper'


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
    maintainer_email='chuyun@todo.todo',
    description='Dynamic semantic map for xuegecar',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dynamic_semantic_mapper = '
            'xuegecar_dynamic_semantic_mapper.dynamic_semantic_mapper:main',
        ],
    },
)
