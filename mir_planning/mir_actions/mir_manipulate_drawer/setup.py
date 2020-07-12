#!/usr/bin/env python

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["mir_manipulate_drawer"],
    package_dir={"mir_manipulate_drawer": "ros/src/mir_manipulate_drawer"},
)

setup(**d)
