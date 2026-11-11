from setuptools import setup
from glob import glob
import os

package_name = 'sensors_arm_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sensors-arm',
    maintainer_email='dev@local',
    description='Bringup launches for sensors-arm ROS2',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
