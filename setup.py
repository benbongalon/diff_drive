import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'diff_drive'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name), glob('launch/*.launch')),
        (os.path.join('share', package_name), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ben Bongalon',
    maintainer_email='ben.bongalon@gmail.com',
    description='Controller for a differential drive robot.\n' \
                'ROS 2 port of diff_drive (https://github.com/merose/diff_drive)',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = diff_drive.diff_drive_controller:main',
            'odometry = diff_drive.diff_drive_odometry:main',
            'mock_robot = diff_drive.diff_drive_mock_robot:main',
            'go_to_goal = diff_drive.diff_drive_go_to_goal:main'
        ],
    },
)
