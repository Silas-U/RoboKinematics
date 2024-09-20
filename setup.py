from setuptools import setup, find_packages

VERSION = '1.0' 
DESCRIPTION = 'A Python package designed to perform kinematic analysis'
LONG_DESCRIPTION = 'A Python package designed to perform kinematic analysis for an n-degree-of-freedom robot manipulator'

# Setting up
setup(
       # the name must match the folder name 'verysimplemodule'
        name="RoboKinematics", 
        version=VERSION,
        author="Silas Udofia",
        author_email="<silasudofia469@gmail.com>",
        description=DESCRIPTION,
        long_description=LONG_DESCRIPTION,
        packages=find_packages(),

        install_requires=[
            'numpy',
            'scipy',
            'matplotlib'
        ],

        keywords=['python', 'kinematics package'],

        classifiers= [
            "Programming Language :: Python :: 2",
            "Programming Language :: Python :: 3",
            "Operating System :: MacOS :: MacOS X",
            "Operating System :: Microsoft :: Windows",
            "Operating System :: POSIX :: Linux"
        ]
)
