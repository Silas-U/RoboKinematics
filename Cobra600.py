"""
Author: Silas Udofia
Date: 2024-08-02
Description: This script performs kinematics analysis for a SCARA-Cobra600 robot.

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
from math import pi as π
from time import sleep
from timeit import default_timer as timer


# Creates a kinematic model of the SCARA robot
Cobra600 = CreateKinematicModel(
    [
        ("frame0", "r", 0.325,  0.0,  0.387, 0.0),
        ("frame1", "r", 0.275,  π,    0.0,   0.0),
        ("frame2", "p", 0.0,    0.0,  0.0,   0.0),
        ("frame3", "r", 0.0,    0.0,  0.0,   0.0),
    ],
    robot_name="Cobra600", link_twist_in_rads=True, #joint_lim_enable=True
)


"""
If joint vars are provided in radians then the min and max limit should be set in radians: 
Default min and max limits are set in degrees
"""
Cobra600.set_joint_limit(
    [
        (-90, 90), #min max j1
        (-90, 90), #min max j2
        (0, 5), #min max j3
        (-90, 90), #min max j4
    ]
)

qr = Cobra600.set_joints([0, 0, 0, 0]) # 60, 30, 0.2, 0
t = Cobra600.f_kin(qr)
start = timer()
p= Cobra600.i_kin([0.1625, 0.55645826, 0.187,  3.14159265, 0, 1.57079633]) # x,y,z,roll,pitch,yaw
print(p)
end = timer()
print('It took %.5f s. to execute.' % (end - start)) 


