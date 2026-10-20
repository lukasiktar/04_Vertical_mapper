from glob import glob

from setuptools import setup

package_name = 'd455_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luka Siktar',
    maintainer_email='luka.siktar11@gmail.com',
    description='RealSense D455 mapping/relocalization with cuVSLAM, RTAB-Map and nvblox.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mesh_marker_publisher = d455_mapping.mesh_marker_publisher:main',
            'prior_map_publisher = d455_mapping.prior_map_publisher:main',
            'px4_bag_odom = d455_mapping.px4_bag_odom:main',
            'emitter_check = d455_mapping.emitter_check:main',
            'emitter_ensure = d455_mapping.emitter_ensure:main',
            'odom_drift_check = d455_mapping.odom_drift_check:main',
            'cross_detector = d455_mapping.cross_detector:main',
            'pins = d455_mapping.pin_tool:main',
        ],
    },
)
