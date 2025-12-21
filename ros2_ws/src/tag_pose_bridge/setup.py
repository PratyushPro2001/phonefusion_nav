from setuptools import setup

package_name = 'tag_pose_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tag_pose_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pratyush',
    maintainer_email='pratyush402@gmail.com',
    description='Bridge AprilTag detections to a single PoseStamped /tag_pose',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tag_pose_bridge = tag_pose_bridge.tag_pose_bridge_node:main',
        ],
    },
)
