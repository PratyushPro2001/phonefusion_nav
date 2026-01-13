from setuptools import setup

package_name = 'feature_tracker'

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
    description='Subscribe to /camera/image_raw and publish /camera/features with ORB keypoints overlay.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'track = feature_tracker.feature_tracker_node:main',
        ],
    },
)
