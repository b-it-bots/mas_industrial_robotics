# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
            packages=['rgb_object_recognition',
                        'pc_object_recognition'],
            package_dir={'rgb_object_recognition': 'common/src/rgb_object_recognition', 
                        'pc_object_recognition': 'common/src/pc_object_recognition'}
            )

setup(**d)
