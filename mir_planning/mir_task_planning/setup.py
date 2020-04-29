#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['planner_wrapper', 'mir_task_planning'],
   package_dir={'planner_wrapper': 'common/planner_wrapper',
                'mir_task_planning': 'ros/src/mir_task_planning'}
)

setup(**d)
