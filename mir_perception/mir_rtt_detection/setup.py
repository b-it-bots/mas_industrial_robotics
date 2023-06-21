# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
            packages=['rtt_node'],
            package_dir={'rtt_node': 'ros/scripts/rtt_node'},
            )

setup(**d)
