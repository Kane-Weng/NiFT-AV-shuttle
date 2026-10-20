from setuptools import find_packages, setup

package_name = 'nift_common'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['vehicle.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # for xacro: $(find nift_common)
        ('share/' + package_name + '/config', [package_name + '/vehicle.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kane Weng',
    maintainer_email='kaichi@umich.edu',
    description=('Facts every NiFT package shares: vehicle geometry and sensor mounts, '
                 'the CAN protocol, the map origin'),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
