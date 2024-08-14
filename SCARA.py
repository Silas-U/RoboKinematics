"""
Author: Silas Udofia
Date: 2024-08-02
Description: This script performs kinematics analysis for a SCARA robot.

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


'''DH TABLE FOR SAMPLE SCARA ROBOT'''
'''---------+-------------+--------------+---------------+--------------+
|    Link   | Link Length |  Link Twist  |  Joint Offset |     Theta    |
------------+-------------+--------------+---------------+--------------+
|     1     |      a1     |      0 deg   |       d1      |    theta1    |
|     2     |      a2     |      0 deg   |       d2      |    theta2    |
|     3     |      a3     |      0 deg   |       d3      |    theta3    |
+-----------+-------------+--------------+---------------+------------'''

from Libs.RoboKinematics import CreateKinematicModel
import timeit
from math import pi as π
from time import sleep
import os


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

# Creates a kinematic model of the SCARA robot
scara = CreateKinematicModel(
    [
        ("frame0", "r", 0.0,  0.0,  0.4, 0.0),
        ("frame1", "r", 0.14,  π,    0.0, 0.0),
        ("frame2", "p", 0.0,  0.0,  0.0, 0.0),
    ],
    robot_name="SCARA", link_twist_in_rads=True, joint_lim_enable=True
)

"""
If joint vars are provided in radians then the min and max limit should be set in radians: 
Default min and max limits are set in degrees
"""
scara.set_joint_limit(
    [
        (0, 90), #min max j1
        (0, 90), #min max j2
        (0, 0.5),#min max j3
    ]
)

# Set initial joint angles, print A0_3, compute jacobian
for i in range(0,10,1):
    os.system('clear')
    qr = scara.set_joints([i, 10, 0.2])
    scara.f_kin(qr)
    j = scara.jacobian()
    print(j)
    sleep(0.02)
    exec_time("scara.jacobian()")





