#!/usr/bin/env python

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["mir_knowledge_ros", "mercury_planner"],
    package_dir={"mir_knowledge_ros": "ros/src/mir_knowledge_ros"},
)

setup(**d)
