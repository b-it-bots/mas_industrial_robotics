#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mcr_knowledge_ros', 'mercury_planner', 'mcr_knowledge'], 
    package_dir={'mcr_knowledge_ros': 'ros/src/mcr_knowledge_ros', 'mcr_knowledge': 'common/src/mcr_knowledge'}
)

setup(**d)
