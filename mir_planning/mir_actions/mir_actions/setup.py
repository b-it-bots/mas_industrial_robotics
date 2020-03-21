#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mir_actions'],
   package_dir={'mir_actions': 'ros/src/mir_actions'}
)

setup(**d)
