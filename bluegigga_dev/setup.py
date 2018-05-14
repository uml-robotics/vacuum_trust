from distutils.core import setup
with open("requirements.txt", 'rt') as f:
    requirements = f.read().splitlines()

setup(
    version='0.0.0',
#     scripts=['src/myscript'],
    packages=['chil'],
    package_dir={'': 'src'},
    install_requires=requirements

)
