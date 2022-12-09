from setuptools import setup
import os
from glob import glob

package_name = 'mir_gripper_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robocup',
    maintainer_email='vamsikalagaturu@gmail.com',
    description='gripper controller package using teensy board',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'gripper_controller = mir_gripper_controller.teensy_gripper_controller_ros:main',
        ],
    },
)
