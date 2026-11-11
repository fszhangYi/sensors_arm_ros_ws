from setuptools import setup
from glob import glob

package_name = 'sensors_arm_control'

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
    description='sensors-arm control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_joint_driver = sensors_arm_control.sim_joint_driver:main',
            'arm_control_node = sensors_arm_control.arm_control_node:main',
        ],
    },
)
