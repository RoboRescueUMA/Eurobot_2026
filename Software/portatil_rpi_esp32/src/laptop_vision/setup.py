from setuptools import setup
import os
from glob import glob

package_name = 'laptop_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboRescue',
    maintainer_email='team@roborescue.com',
    description='Sistema de visión cenital para detección ArUco - Ejecuta en Laptop',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = laptop_vision.camera_publisher:main',
            'aruco_detector = laptop_vision.aruco_detector:main',
            'aruco_navigator = laptop_vision.aruco_navigator:main',
        ],
    },
)
