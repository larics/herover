import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'herover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='morsag',
    maintainer_email='matko.orsag@gmail.com',
    description='ROS 2 Twist to serial bridge for Arduino mobile robot control',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'twist_serial_bridge = herover.twist_serial_bridge:main',
        ],
    },
)
