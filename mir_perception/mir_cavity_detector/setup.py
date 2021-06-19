# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
            packages=['mcr_cavity_detector'],
            package_dir={'mcr_cavity_detector': 'common/src/mcr_cavity_detector'}
            )

setup(**d)