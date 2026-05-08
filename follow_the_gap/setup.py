from setuptools import setup
import os
from glob import glob

package_name = 'follow_the_gap'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoshi',
    maintainer_email='yoshi@example.com',
    description='Follow the Gap with Pure Pursuit and speed PID',
    license='MIT',
    entry_points={
        'console_scripts': [
            'follow_the_gap_node = follow_the_gap.follow_the_gap_node:main',
        ],
    },
)
