#!/usr/bin/env python

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["mir_world_generation"],
    package_dir={"mir_world_generation": "common/mir_world_generation"},
)

setup(**d)
