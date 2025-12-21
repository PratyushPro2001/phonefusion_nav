from setuptools import setup

package_name = 'gps_anchor_fuser'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gps_anchor_fuser.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pratyush',
    description='Phase 2.5: Simple GPS anchoring for IMU dead reckoning',
    entry_points={
        'console_scripts': [
            'gps_anchor_node = gps_anchor_fuser.gps_anchor_node:main'
        ],
    },
)
