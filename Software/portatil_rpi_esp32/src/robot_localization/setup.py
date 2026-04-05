from setuptools import setup
import os
from glob import glob

package_name = 'robot_localization'
calibration_glob = glob(os.path.join(package_name, 'calibration', '*'))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'calibration'), calibration_glob),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    package_data={
        package_name: ['calibration/*'] if calibration_glob else [],
    },
    zip_safe=True,
    maintainer='RoboRescue',
    maintainer_email='team@roborescue.com',
    description='Localizacion absoluta en campo Eurobot mediante homografia ArUco - Ejecuta en Laptop',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = robot_localization.camera_publisher:main',
            'field_localizer = robot_localization.field_localizer:main',
            'field_navigator = robot_localization.field_navigator:main',
            'cerebro_eurobot = robot_localization.cerebro_eurobot:main',
            'controlador_garra = robot_localization.controlador_garra:main',
            
        ],
    },
)
