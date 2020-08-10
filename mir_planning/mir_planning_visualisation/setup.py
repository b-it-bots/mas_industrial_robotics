#!/usr/bin/env python

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["mir_planning_visualisation"],
    package_dir={"mir_planning_visualisation": "ros/src/mir_planning_visualisation"},
)

setup(**d)
