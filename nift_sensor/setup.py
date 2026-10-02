from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'nift_sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kane Weng',
    maintainer_email='kaichi@umich.edu',
    description='Unified sensor pipeline for the NiFT shuttle',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'beacon_node = nift_sensor.beacon_node:main',
            'lidar_node = nift_sensor.lidar_node:main',
            'shuttle_tf_node = nift_sensor.shuttle_tf_node:main',
        ],
    },
)
