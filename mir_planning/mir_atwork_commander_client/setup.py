#!/usr/bin/env python

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["mir_atwork_commander_client"],
    package_dir={
        "mir_atwork_commander_client": "ros/src/mir_atwork_commander_client",
    },
)

setup(**d)
