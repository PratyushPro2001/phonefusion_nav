from setuptools import setup

package_name = 'phone_vio_lite'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pratyush',
    maintainer_email='pratyush@example.com',
    description='Phase 3B-Lite: IP Webcam visual odometry + ROS2 path publishing',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ipcam_vo_node = phone_vio_lite.ipcam_vo_node:main',
        ],
    },
)
