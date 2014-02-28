## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mir_states_common', 'mir_states'],
    package_dir={'mir_states_common' : 'common/src', 
                 'mir_states' : 'ros/src'}
)

setup(**d)

