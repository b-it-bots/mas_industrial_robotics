# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
            packages=['mir_object_recognition','pcl_classifier'],
            package_dir={'mir_object_recognition': 'common/src/mir_object_recognition', 'pcl_models': 'common/src/pcl_models'}
            )

setup(**d)
