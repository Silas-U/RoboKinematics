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
import timeit

"""
The Denavit–Hartenberg parameters for UR20  robot is shown below.

The Denavit–Hartenberg parameters for UR20  robot is shown below.

Kinematics  theta [rad]     a [m]   d [m]   alpha [rad]
Joint 1     0               0       0.6718      π/2
Joint 2     0               0.4318    0           0
Joint 3     0               0.0203  0.15       -π/2
Joint 4     0               0       0.4318      π/2
Joint 5     0               0       0          -π/2
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

# Creates a kinematic model of the Puma560 robot
Puma560 = CreateKinematicModel(
    [
        ("frame0", "r", 0,       90,    0.6718, 0.0),
        ("frame1", "r", 0.4318,   0,    0,      0.0),
        ("frame2", "r", 0.0203, -90,    0.15,   0.0),
        ("frame3", "r", 0,       90,    0.4318, 0.0),
        ("frame4", "r", 0,      -90,    0.0,    0.0),
        ("frame5", "r", 0,        0,    0.0,    0.0)
    ]
    , robot_name="Puma560")

# Set initial joint angles, print A0_6, compute jacobian
Puma560.set_joints([0.1, 0.2, 0.3, 0.4, 0.5, 0.6],rads=True)
Puma560.print_transforms(6)
j = Puma560.jacobian()
print(j)
