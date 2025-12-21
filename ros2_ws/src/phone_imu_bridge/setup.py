from setuptools import setup

package_name = 'phone_imu_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/phone_imu_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pratyush',
    description='UDP phone IMU -> sensor_msgs/Imu bridge',
    entry_points={
        'console_scripts': [
            'phone_imu_bridge_node = phone_imu_bridge.phone_imu_bridge_node:main'
        ],
    },
)
