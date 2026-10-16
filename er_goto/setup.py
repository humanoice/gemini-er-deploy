# SPDX-License-Identifier: GPL-3.0-only
from glob import glob

from setuptools import setup

package_name = 'er_goto'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Son',
    maintainer_email='33903086+sitthaveet@users.noreply.github.com',
    description='Walk the RPO humanoid to an object located by Gemini Robotics-ER.',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'er_goto_node = er_goto.node:main',
        ],
    },
)
