from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="RoboKinematics",
    version="0.0.2",
    author="Silas Udofia",
    author_email="silasudofia469@gmail.com",
    description="A Python library for robotics research and education",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/Silas-U/Robot-Kinematics-lib/tree/main",
    project_urls={
        "Documentation": "https://yourusername.github.io/your-repo",
        "Bug Tracker": "https://github.com/Silas-U/Robot-Kinematics-lib/tree/main/issues",
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
