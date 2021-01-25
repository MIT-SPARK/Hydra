## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['kimera_scene_graph_python'],
    package_dir={'': 'kimera_scene_graph_python'})

setup(**setup_args)