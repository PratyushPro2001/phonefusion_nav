from setuptools import find_packages, setup

package_name = 'ipcam_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pratyush',
    maintainer_email='pratyush402@gmail.com',
    description='Phone IP camera -> ROS2 /camera/image_raw publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ipcam_pub = ipcam_bridge.ipcam_pub:main',
        ],
    },
)
