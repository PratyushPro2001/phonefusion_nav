from setuptools import setup

package_name = 'imu_orientation_viz'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/imu_orientation_viz.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@todo.com',
    description='Real-time 3D orientation visualization using IMU data',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'imu_orientation_viz = imu_orientation_viz.imu_orientation_viz:main'
        ],
    },
)
