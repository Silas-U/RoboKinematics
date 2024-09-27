from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="RoboKinematics",
    version="1.0.1",
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
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent",
    ],
    packages=find_packages(),
    python_requires='>=3.6',
    install_requires=[  
        "numpy",
        "scipy",
        "matplotlib"
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
