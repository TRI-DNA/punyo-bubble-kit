from setuptools import setup
import os
from glob import glob

package_name = 'punyo'

# Using a submodule format
# https://answers.ros.org/question/367793/including-a-python-module-in-a-ros2-package/
submodules = "punyo/utils"

setup(
    name=package_name,
    version='0.9.1',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('punyo/config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('punyo/models/*.pth')),
        (os.path.join('share', package_name, 'sounds'), glob('punyo/sounds/*.wav')),
        (os.path.join('share', package_name), glob('launch/*_launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Edwardo Martinez',
    maintainer_email='edwardo.martinez@tri.global',
    description='ROS2 package for the Punyo Soft-Bubble Sensor',
    license='Punyo Soft-Bubble Sensor - Copyright 2023 Toyota Research Institute. All rights reserved.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	        'bubble_image_subscriber = punyo.bubble_image_sub:main',
            'bubble_sensor_subscriber = punyo.bubble_sensor_sub:main',
            'bubble_music_subscriber = punyo.bubble_music_sub:main',
            'bubble_control = punyo.bubble_sensor_control:main',
        ],
    },
)
