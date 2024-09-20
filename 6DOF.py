"""
Author: Silas Udofia
Date: 2024-08-02
Description: This script performs kinematics analysis for a 6DOF robot.

GitHub: https://github.com/Silas-U/Robot-Kinematics-lib/tree/main

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

from Libs.RoboKinematics import CreateKinematicModel
from timeit import default_timer as timer
import numpy as np
import matplotlib.pyplot as plt

"""
The Denavit–Hartenberg parameters for 6DOF  robot is shown below.

Kinematics  theta [rad]     a [m]   d [m]   alpha [rad]
Joint 1     0               0.05    0.105       π/2
Joint 2     0               0.14    0           0
Joint 3     0               0.17    0           π/2
Joint 4     0               0       0          -π/2
Joint 5     0               0       0           π/2
Joint 6     0               0       0           0
"""

robot = CreateKinematicModel(
    [
        { 'frame_name': 'frame0', 'joint_type': 'r', 'link_length': 0.05, 'twist': 90.0, 'offset': 0.105, 'theta': 0.0 },
        { 'frame_name': 'frame1', 'joint_type': 'r', 'link_length': 0.14, 'twist': 0.0,  'offset': 0.0, 'theta': 0.0 },
        { 'frame_name': 'frame2', 'joint_type': 'r', 'link_length': 0.17, 'twist': 90.0, 'offset': 0.0, 'theta': 0.0 },
        { 'frame_name': 'frame3', 'joint_type': 'r', 'link_length': 0.0,  'twist': -90.0,'offset': 0.0, 'theta': 0.0 },
        { 'frame_name': 'frame4', 'joint_type': 'r', 'link_length': 0.0,  'twist': 90.0, 'offset': 0.0, 'theta': 0.0 },
        { 'frame_name': 'frame5', 'joint_type': 'r', 'link_length': 0.0,  'twist': 0.0,  'offset': 0.02,'theta': 0.0 },
    ],
    robot_name="6DOF")

way_points  = [
    [3.60000000e-01, -2.44929360e-18,  8.50000000e-02, 3.14159265e+00,  0.00000000e+00,  0.00000000e+00],
    [2.20000000e-01,  6.12323400e-18,  2.25000000e-01, 3.14159265e+00,  0.00000000e+00, -5.23598776e-01],
    [0.16,   0.24248711,  0.21624356,-2.15879893, -0.4478324,  -1.8133602],
    [2.20000000e-01,  6.12323400e-18,  2.25000000e-01, 3.14159265e+00,  0.00000000e+00, -5.23598776e-01],
]

trj_time = [1,2,3]

robot.f_kin([0, 0, 0, 0, 0, 0])
joint_space = [robot.i_kin(way_point,it_max=100) for way_point in way_points]                                                                          
trajectory = robot.traj_gen(joint_space, trj_time, 0, plot=True)

