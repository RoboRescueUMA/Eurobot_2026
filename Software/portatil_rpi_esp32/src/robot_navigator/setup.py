from setuptools import find_packages, setup

package_name = 'robot_navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='axarbot',
    maintainer_email='ivanmorladag@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'zenital_publisher = robot_navigator.zenital_publisher:main',
		'aruco_detector = robot_navigator.aruco_detector:main',
		'aruco_detector_mov = robot_navigator.aruco_detector_mov:main',  
		'aruco_controller = robot_navigator.aruco_controller:main',
      ],
    },
)
