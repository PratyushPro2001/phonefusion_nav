from setuptools import setup

package_name = 'vio_fuser'

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
    description='Fuse VO position with IMU orientation and publish fused odom/path.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fuse = vio_fuser.imu_fuse_node:main',
        ],
    },
)
