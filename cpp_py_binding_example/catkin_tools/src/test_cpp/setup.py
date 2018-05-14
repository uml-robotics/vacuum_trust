from distutils.core import setup
setup(
    version= '0.0.0',
    packages=['test_cpp'],
    package_dir={'': 'src'}
)
# from catkin_pkg.python_setup import generate_distutils_setup
# 
# setup_args = generate_distutils_setup(
#   packages=['test_cpp'],
#   package_dir={'': 'src'},
# )
# 
# setup(**setup_args)
