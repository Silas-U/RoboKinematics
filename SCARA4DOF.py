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
from Libs.RoboKinematics import CreateKinematicModel

'''DH TABLE FOR SAMPLE SCARA ROBOT'''
'''---------+-------------+--------------+---------------+--------------+
|    Link   | Link Length |  Link Twist  |  Joint Offset |     Theta    |
------------+-------------+--------------+---------------+--------------+
|     1     |      a1     |      0 deg   |       d1      |    theta1    |
|     2     |      a2     |      0 deg   |       d2      |    theta2    |
|     3     |      a3     |      0 deg   |       d3      |    theta3    |
|     4     |      a4     |      0 deg   |       d4      |    theta4    |
+-----------+-------------+--------------+---------------+------------'''

# Creates a kinematic model of the SCARA robot
scara = CreateKinematicModel(
    [   
        { 'frame_name': 'frame0', 'joint_type': 'r', 'link_length': 0.325, 'twist': 0.0, 'offset': 0.387, 'theta': 0.0 },
        { 'frame_name': 'frame1', 'joint_type': 'r', 'link_length': 0.275, 'twist': 3.142,'offset': 0.0, 'theta': 0.0 },
        { 'frame_name': 'frame2', 'joint_type': 'p', 'link_length': 0.0, 'twist': 0.0, 'offset': 0.0, 'theta': 0.0 },
        { 'frame_name': 'frame3', 'joint_type': 'r', 'link_length': 0.0, 'twist': 0.0, 'offset': 0.0, 'theta': 0.0 }
    ],
    robot_name="SCARA", link_twist_in_rads=True,  joint_lim_enable=True
)

scara.set_joint_limit(
    [
        [-90, 90],
        [-90, 90],
        [0, 5],
        [0, 90]
    ]
)

trj_time = [1]
scara.f_kin([30, 20, 0, 0])
home = scara.get_joint_states(rads=True)
target_1 = scara.i_kin([0.325, 0.275, 0.387, 0, 0, 1.39626342], mask=[1, 1, 1, 0, 0, 1])

jq = [
    home,
    target_1,
]

trajectory = scara.traj_gen(jq, trj_time, 0, plot=True)
