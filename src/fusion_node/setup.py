from setuptools import setup

package_name = 'fusion_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/fusion_node.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@todo.com',
    description='IMU + GPS (+ AprilTag) fusion to produce /fusion/odom',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fusion_node = fusion_node.fusion_node:main'
        ],
    },
)
