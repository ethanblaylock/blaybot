from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mobility'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools', 'sensor_msgs', 'rclpy', 'std_msgs'],
    zip_safe=True,
    maintainer='base',
    maintainer_email='ethan.blaylock37@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_node = mobility.drive_node:main',
        ],
    },

)
