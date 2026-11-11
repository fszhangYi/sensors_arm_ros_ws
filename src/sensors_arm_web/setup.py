from setuptools import setup
from glob import glob

package_name = 'sensors_arm_web'

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
    install_requires=['setuptools', 'fastapi', 'uvicorn', 'pyyaml', 'websockets'],
    zip_safe=True,
    maintainer='sensors-arm',
    maintainer_email='dev@local',
    description='sensors-arm ROS2 web UI',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'web_server = sensors_arm_web.web_server:main',
        ],
    },
)
