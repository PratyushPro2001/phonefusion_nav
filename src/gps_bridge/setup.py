from setuptools import setup

package_name = 'gps_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gps_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@todo.com',
    description='Bridge for phone GPS (UDP/JSON) to ROS2 /gps/fix and /gps/pose',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gps_bridge = gps_bridge.gps_bridge_node:main'
        ],
    },
)
