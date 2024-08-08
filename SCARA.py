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

from Libs.RoboKinematics import CreateRobot
import timeit
from math import pi as π
from time import sleep



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


#[Frame_name, Joint_type, link_length, link_twist, joint_offset, joint_variable] the joint_variables are initialized to 0.0 here]
dh_params_link1 = ["frame0", "r", 0.0,  0.0,  0.4, 0.0]
dh_params_link2 = ["frame1", "r", 0.14, π,    0.0, 0.0]
dh_params_link3 = ["frame2", "p", 0.0,  0.0,  0.0, 0.0]


dh_params = [
    dh_params_link1,
    dh_params_link2,
    dh_params_link3,
]

"""
If joint vars are provided in radians then the min and max limit should be set in radians: 
Default min and max limits are set in degrees
"""
joint_lim = [
    [-π/2, π/2], #min max j1
    [-π/2, π/2], #min max j2
    [0, 0.5],    #min max j3
]

scara_arm = CreateRobot(dh_params, robot_name="SCARA", link_twist_in_rads=True, joint_lim_enable=True)

scara_arm.set_joint_limit(joint_lim)

#Moves the robot to home position
scara_arm.move_joints([π/2, π/2, 0.5],rads=True)
scara_arm.print_transforms(3)
sleep(3)

# The following code can make the SCARA arm swing left and right

scara_arm.move_joints([π/3, 0, 0.0],rads=True)
scara_arm.print_transforms(3)
print("\n")
sleep(0.5)
scara_arm.move_joints([0, 0, 0.0],rads=True)
scara_arm.print_transforms(3)
print("\n")
sleep(0.5)
scara_arm.move_joints([π/3, 0, 0.0],rads=True)
scara_arm.print_transforms(3)
print("\n")
sleep(0.5)
scara_arm.move_joints([0, 0, 0.0],rads=True)
scara_arm.print_transforms(3)
print("\n")
sleep(0.5)


