#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mir_moveit_scene_ros'],
    package_dir={'mir_moveit_scene_ros': 'ros/src/mir_moveit_scene_ros'}
)

setup(**d)
