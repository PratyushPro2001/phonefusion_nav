from setuptools import setup

package_name = 'imu_deadreckoning'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name + '/launch',
         ['launch/imu_deadreckoning.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pratyush',
    description='IMU-only dead reckoning',
    entry_points={
        'console_scripts': [
            'imu_deadreckoning_node = imu_deadreckoning.imu_deadreckoning_node:main'
        ],
    },
)
