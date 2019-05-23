# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
            packages=['rgb_object_recognition',
                        'pcl_classifiers',
                        'rgb_classifiers'],
            package_dir={'rgb_object_recognition': 'common/src/rgb_object_recognition', 
                        'pcl_classifiers': 'common/src/pcl_classifiers',
                        'rgb_classifiers': 'common/src/rgb_classifiers'}
            )

setup(**d)
