"""
Author: Silas Udofia
Date: 2024-08-02
Description: This script performs kinematics analysis for a UR20 robot.

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
from time import sleep
from math import pi

"""
The Denavit–Hartenberg parameters for UR20  robot is shown below.

Kinematics  theta [rad]   a [m]     d [m]   alpha [rad]
Joint 1     0               0       0.2363      π/2
Joint 2     0              -0.8620  0           0
Joint 3     0              -0.7287  0           0
Joint 4     0               0       0.2010      π/2
Joint 5     0               0       0.1593     -π/2
Joint 6     0               0       0.1543      0
"""

# [Frame_name, Joint_type, link_length, link_twist, joint_offset, joint_variable]

dh_params_link1 = ["frame0", "r", 0,        pi/2,  0.2363, 0]
dh_params_link2 = ["frame1", "r", -0.8620,   0,    0,      0]
dh_params_link3 = ["frame2", "r", -0.7287,   0,    0,      0]
dh_params_link4 = ["frame3", "r", 0,        pi/2,  0.2010, 0]
dh_params_link5 = ["frame4", "r", 0,       -pi/2,  0.1593, 0]
dh_params_link6 = ["frame5", "r", 0,        0,     0.1543, 0]


dh_params = [
    dh_params_link1,
    dh_params_link2,
    dh_params_link3,
    dh_params_link4,
    dh_params_link5,
    dh_params_link6,
    ]

joint_lim = [
    [-pi/2, pi/2],  # min max j1
    [-pi/2, pi/2],  # min max j2
    [-pi/2, pi/2],  # min max j1
    [-pi/2, pi/2],  # min max j2
    [-pi/2, pi/2],  # min max j1
    [-pi/2, pi/2],  # min max j2
]

ur20 = CreateRobot(dh_params, "UR20", link_twist_in_rads=True, joint_lim_enable=True)

ur20.set_joint_limit(joint_lim)


def exec_time(f):
    print(
        'Execution time : '    
        '{:.10f} s'.format(
            timeit.timeit(
                f,
                globals=globals(),
                number=1,
            )
        )
     )


# Move joints in rads
ur20.move_joints([0, -pi/2, -pi/2, 0, 0, 0], rads=True)
ur20.print_transforms(6)
print("\n")
ur20.genJacobian()
