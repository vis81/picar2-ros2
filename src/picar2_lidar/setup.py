import os
from glob import glob
from setuptools import setup

package_name = 'picar2_lidar'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Valentyn Shevchenko',
    maintainer_email='valentin.shevchenko@gmail.com',
    description='LDS02RR LiDAR driver with motor PI control for picar2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lidar_node = picar2_lidar.lidar_node:main',
        ],
    },
)
