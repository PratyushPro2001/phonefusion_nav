import os
from glob import glob
from setuptools import setup

package_name = 'phone_ekf'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='PhoneFusion-Nav',
    maintainer_email='wecarevts@gmail.com',
    description='Phone EKF',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'ekf_node = phone_ekf.ekf_node:main',
        'run_offline = phone_ekf.run_offline:main',
    ]},
)
