from distutils.core import setup

setup(
    version='0.0.0',
    scripts=['src/vacuum_controllers/robot_controller.py'],
    packages=['vacuum_controllers'],
    package_dir={'': 'src'}
)

