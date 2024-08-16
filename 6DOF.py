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
import os
from Libs.RoboKinematics import CreateKinematicModel
import timeit
from math import pi
from time import sleep
from timeit import default_timer as timer

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
        ("frame0", "r", 0.05,  90,   0.0,   0.0),
        ("frame1", "r", 0.14,  0,    0.0,   0.0),
        ("frame2", "r", 0.17,  90,   0.0,   0.0),
        ("frame3", "r", 0.0,  -90,   0.0,   0.0),
        ("frame4", "r", 0.0,   90,   0.0,   0.0),
        ("frame5", "r", 0.0,   0,    0.0,   0.0)
    ]
    , robot_name="6DOF", joint_lim_enable=True)


robot.set_joint_limit(
    [
        (-90, 90),  # min max j1
        (-90, 90),  # min max j2
        (-90, 90),  # min max j3
        (-90, 90),  # min max j4
        (-90, 90),  # min max j5
        (-90, 90),  # min max j6
    ]
)


#Set initial joint angles, print A0_6, compute jacobian

for i in range(0,90,1):
    os.system('clear')
    start = timer()
    q = robot.set_joints([0, 0, -90, 0, i, 0])
    t = robot.f_kin(q)
    tcp = robot.get_tcp()
    j = robot.jacobian()
    end = timer()

    print(t,"\n")
    print(tcp,"\n")
    print(j,"\n")
    print('It took %.5f s. to execute.' % (end - start)) 
    sleep(0.02)
