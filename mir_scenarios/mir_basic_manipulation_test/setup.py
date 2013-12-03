#!/usr/bin/env python

import distutils.core
import catkin_pkg.python_setup

d = catkin_pkg.python_setup.generate_distutils_setup(
   packages=['mir_basic_manipulation_test'],
   package_dir={'mir_basic_manipulation_test': 'ros/src'}
)

distutils.core.setup(**d)
