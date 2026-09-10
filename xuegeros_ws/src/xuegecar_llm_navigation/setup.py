import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'xuegecar_llm_navigation'

setup(
    name=package_name,
    version='0.1.0',
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
    description='P5 LLM semantic query + navigation module for xuegecar '
                '(DeepSeek intent parsing, deterministic query engine, '
                'Nav2 goal generation with clearance audit)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'llm_navigation_node = '
            'xuegecar_llm_navigation.llm_navigation_node:main',
            'llm_console = '
            'xuegecar_llm_navigation.llm_console:main',
            'fake_dynamic_map = '
            'xuegecar_llm_navigation.tools.fake_dynamic_map:main',
            'fake_nav2_server = '
            'xuegecar_llm_navigation.tools.fake_nav2_server:main',
            'mock_deepseek = '
            'xuegecar_llm_navigation.tools.mock_deepseek:main',
        ],
    },
)
