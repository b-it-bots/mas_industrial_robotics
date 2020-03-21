#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mir_move_base_ros'],
    package_dir={'mir_move_base_ros': 'ros/src/mir_move_base_ros'}
)

setup(**d)
