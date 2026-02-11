from setuptools import setup

package_name = 'rpi_relay'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team RoboRescue',
    maintainer_email='team@roborescue.com',
    description='Relay node to forward commands from laptop to ESP32 - Runs on RPI',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_relay = rpi_relay.cmd_vel_relay:main',
        ],
    },
)
