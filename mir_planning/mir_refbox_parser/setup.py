#!/usr/bin/env python

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["mir_refbox_parser_ros"],
    package_dir={"mir_refbox_parser_ros": "ros/src/mir_refbox_parser_ros"},
)

setup(**d)
