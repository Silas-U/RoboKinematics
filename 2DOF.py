"""
Author: Silas Udofia
Date: 2024-08-02
Description: This script performs kinematics analysis for a 2DOF robot.

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


'''DH TABLE FOR SAMPLE 2DOF ROBOT'''
'''---------+-------------+--------------+---------------+--------------+
|    Link   | Link Length |  Link Twist  |  Joint Offset |     Theta    |
------------+-------------+--------------+---------------+--------------+
|     1     |      a1     |      0 deg   |       d1      |    theta1    |
|     2     |      a2     |      0 deg   |       d2      |    theta2    |
+-----------+-------------+--------------+---------------+------------'''

import os
from Libs.RoboKinematics import CreateKinematicModel
from math import pi as π
from time import sleep
from timeit import default_timer as timer


# Creates a kinematic model of the SCARA robot
scara = CreateKinematicModel(
    [
        ("frame0", "r", 1.0, 0.0,  0.0, 0.0),
        ("frame1", "r", 1.0, 0.0,  0.0, 0.0),
    ],
    robot_name="SCARA", link_twist_in_rads=True, joint_lim_enable=True
)

"""
If joint vars are provided in radians then the min and max limit should be set in radians: 
Default min and max limits are set in degrees
"""
scara.set_joint_limit(
    [
        (-90, 90), #min max j1
        (-90, 90), #min max j2
    ]
)

qr = scara.set_joints([20, 0])
t = scara.f_kin(qr)
start = scara.get_joint_states(rads=True)
start_t = timer()
goal = scara.i_kin([1.12972504, 1.61341457, 0, 0, 0, 1.13446401])
traj = scara.ptraj(start, goal, 5, 0)
end = timer()
scara.plot(traj)
print('It took %.5f s. to execute.' % (end - start_t)) 

