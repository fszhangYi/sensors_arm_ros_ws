from setuptools import setup
from glob import glob

package_name = 'sensors_arm_hw'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sensors-arm',
    maintainer_email='dev@local',
    description='sensors-arm hardware bridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hw_bridge_node = sensors_arm_hw.hw_bridge_node:main',
        ],
    },
)
