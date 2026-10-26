from setuptools import setup
import os
from glob import glob

package_name = 'sensor_logger_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PhoneFusion-Nav',
    maintainer_email='wecarevts@gmail.com',
    description='Sensor Logger HTTP-push bridge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge = sensor_logger_bridge.bridge_node:main',
            'replay_capture = sensor_logger_bridge.replay:main',
        ],
    },
)
