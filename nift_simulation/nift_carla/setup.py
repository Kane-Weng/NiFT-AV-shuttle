from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'nift_carla'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (
            os.path.join('share', package_name, 'config', 'scenarios'),
            glob('config/scenarios/*.yaml'),
        ),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'docker'), glob('docker/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kane Weng',
    maintainer_email='kaichi@umich.edu',
    description='CARLA and kinematic simulation backends for the NiFT shuttle',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'carla_bridge_node = nift_carla.carla_bridge_node:main',
            'carla_scenario_node = nift_carla.carla_scenario_node:main',
            'kinematic_sim_node = nift_carla.kinematic_sim_node:main',
            'carla_smoke_test = nift_carla.tools.smoke_test:main',
            'carla_pick_spawn = nift_carla.tools.pick_spawn:main',
            'carla_draw_lanelets = nift_carla.tools.draw_lanelets:main',
            'carla_check_alignment = nift_carla.tools.check_alignment:main',
            'carla_eval_route = nift_carla.tools.eval_route:main',
            'carla_camera_view = nift_carla.tools.camera_view:main',
            'carla_step_response = nift_carla.tools.step_response:main',
            'static_waypoint_publisher = nift_carla.tools.static_waypoint_publisher:main',
            'sim_route_run = nift_carla.tools.route_run:main',
            'sim_route_metrics = nift_carla.tools.route_metrics:main',
            'sim_route_plot = nift_carla.tools.route_plot:main',
            'sim_gnss_bench = nift_carla.tools.gnss_bench:main',
        ],
    },
)
