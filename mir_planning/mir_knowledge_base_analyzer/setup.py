#!/usr/bin/env python

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
    packages=["mir_knowledge_base_analyzer", "mir_knowledge_base_analyzer_ros"],
    package_dir={
        "mir_knowledge_base_analyzer": "common/src/mir_knowledge_base_analyzer",
        "mir_knowledge_base_analyzer_ros": "ros/src/mir_knowledge_base_analyzer_ros",
    },
)

setup(**d)
