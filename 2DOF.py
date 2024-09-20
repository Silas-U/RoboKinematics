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

from Libs.RoboKinematics import CreateKinematicModel
from timeit import default_timer as timer
import numpy as np

'''DH TABLE FOR SAMPLE 2DOF ROBOT'''
'''---------+-------------+--------------+---------------+--------------+
|    Link   | Link Length |  Link Twist  |  Joint Offset |     Theta    |
------------+-------------+--------------+---------------+--------------+
|     1     |      a1     |      0 deg   |       d1      |    theta1    |
|     2     |      a2     |      0 deg   |       d2      |    theta2    |
+-----------+-------------+--------------+---------------+------------'''


from Libs.RoboKinematics import CreateKinematicModel

# Define Denavit-Hartenberg (DH) parameters for a 2-joint robot
dh_params = [
    {"frame_name": "link1", "joint_type": "r", "link_length": 1.0, "twist": 0, "offset": 0, "theta": 0},
    {"frame_name": "link2", "joint_type": "r", "link_length": 1.0, "twist": 0, "offset": 0, "theta": 0}
]

# Create the kinematic model
robot = CreateKinematicModel(dh_params, robot_name="2DOF Robot")

# Perform forward kinematics
joint_angles = [90, 0]
robot.f_kin(joint_angles)

# Target position of the end-effector
target_position = [0.96592583, 1.67303261, 0, 0, 0, 1.30899694]

# Generate a trajectory from an initial to a final joint configuration
initial = robot.get_joint_states(rads=True)
final = robot.i_kin(target_position)

jt = [
       initial,
       final 
    ]

trajectory = robot.traj_gen(jt, trj_time=[1], pva=2, plot=True)