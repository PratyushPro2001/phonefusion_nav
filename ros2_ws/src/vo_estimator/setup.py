from setuptools import setup

package_name = 'vo_estimator'

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
    description='Minimal monocular VO from /camera/image_raw -> /vio/odom and /vio/path with strong motion gating.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vo = vo_estimator.vo_node:main',
        ],
    },
)
