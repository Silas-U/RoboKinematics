from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="RoboKinematics",
    version="1.0.2",
    author="Silas Udofia (Silas-U)",
    author_email="silasudofia469@gmail.com",
    description="A Python library for robotics research and education",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/Silas-U/RoboKinematics/tree/main",
    project_urls={
        "Bug Tracker": "https://github.com/Silas-U/RoboKinematics/issues",
    },
    classifiers=[
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent",
         # Specify the Python versions you support here.
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
    packages=find_packages(),
    python_requires='>=3.7',
    install_requires=[  
        "numpy>=1.17.4",
        "matplotlib>=3.1",
        "scipy",
    ],
    include_package_data=True,
    license="Apache Version 2.0",
    keywords=[
        "python",
        "robotics",
        "robokinematics",
        "kinematics",
        "trajectory-generation",
        "jacobian",
        "robot-manipulator",
    ]

)
