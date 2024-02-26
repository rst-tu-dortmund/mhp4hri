## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['spl','spl.data','spl.model','common','common.constants'],
    package_dir={'': 'extern/spl'},
)

setup(**setup_args)
