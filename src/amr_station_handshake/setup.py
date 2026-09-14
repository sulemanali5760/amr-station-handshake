from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_station_handshake'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Suleman Ali',
    maintainer_email='suleman.ali5760@gmail.com',
    description='AMR to conveyor station handover over ROS 2 and OPC UA.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'station_sim = amr_station_handshake.station_sim:main',
            'station_bridge = amr_station_handshake.station_bridge:main',
            'amr_node = amr_station_handshake.amr_node:main',
        ],
    },
)
