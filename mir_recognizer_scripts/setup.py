"""
Copyright 2022 Bonn-Rhein-Sieg University

Author: Vivek Mannava

"""
from setuptools import setup
import os
from glob import glob

package_name = 'mir_recognizer_scripts'
submodules_yolov5 = 'mir_recognizer_scripts/rgb_object_recognition/yolov5'
submodules_utils = 'mir_recognizer_scripts/rgb_object_recognition/yolov5/utils'
submodules_models = 'mir_recognizer_scripts/rgb_object_recognition/yolov5/models'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules_yolov5, submodules_utils, submodules_models],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vivek',
    maintainer_email='vivek.mannava@smail.inf.h-brs.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rgb_recognizer = mir_recognizer_scripts.rgb_object_recognizer_node:main' 
        ],
    },
)
