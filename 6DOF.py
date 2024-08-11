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
from Libs.RoboKinematics import CreateRobot
import timeit
from math import pi

"""
The Denavit–Hartenberg parameters for UR20  robot is shown below.

The Denavit–Hartenberg parameters for UR20  robot is shown below.

Kinematics  theta [rad]     a [m]   d [m]   alpha [rad]
Joint 1     0               0.05    0.105       π/2
Joint 2     0               0.14    0           0
Joint 3     0               0.17    0           π/2
Joint 4     0               0       0          -π/2
Joint 5     0               0       0           π/2
Joint 6     0               0       0           0
"""


def exec_time(f):
    print( 
        'Execution time : ',    
        '{:.10f} sec'.format(
            timeit.timeit(
                f,
                globals=globals(),
                number=1,
            )
        )
    )


# [Frame_name, Joint_type, link_length, link_twist, joint_offset, joint_variable]
dh_params_link1 = ["frame0", "r", 0.05,  90,   0.0,   0.0]
dh_params_link2 = ["frame1", "r", 0.14,  0,    0.0,   0.0]
dh_params_link3 = ["frame2", "r", 0.17,  90,   0.0,   0.0]
dh_params_link4 = ["frame3", "r", 0.0,  -90,   0.0,   0.0]
dh_params_link5 = ["frame4", "r", 0.0,   90,   0.0,   0.0]
dh_params_link6 = ["frame5", "r", 0.0,   0,    0.0,   0.0]


dh_params = [
    dh_params_link1,
    dh_params_link2,
    dh_params_link3,
    dh_params_link4,
    dh_params_link5,
    dh_params_link6,
    ]


robot = CreateRobot(dh_params, "6DOF")

# Move joints in degrees
robot.move_joints([0, 90, -90, 0, 0, 0])
robot.print_transforms(6)
print("\n")
robot.compute_jacobian()
