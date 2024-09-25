"""
Author: Silas Udofia
Date: 2024-08-02
Description: This script performs kinematics analysis for a 6DOF robot.

GitHub: "https://github.com/Silas-U/RoboKinematics/tree/main"

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

from RoboKinematics import CreateKinematicModel
from os import system
from time import sleep

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
        { 'frame_name': 'frame0', 'joint_type': 'r', 'link_length': 0.05, 'twist': 90.0, 'offset': 0.105, 'theta': 0.0 },
        { 'frame_name': 'frame1', 'joint_type': 'r', 'link_length': 0.14, 'twist': 0.0,  'offset': 0.0, 'theta': 0.0 },
        { 'frame_name': 'frame2', 'joint_type': 'r', 'link_length': 0.17, 'twist': 90.0, 'offset': 0.0, 'theta': 0.0 },
        { 'frame_name': 'frame3', 'joint_type': 'r', 'link_length': 0.0,  'twist': -90.0,'offset': 0.0, 'theta': 0.0 },
        { 'frame_name': 'frame4', 'joint_type': 'r', 'link_length': 0.0,  'twist': 90.0, 'offset': 0.0, 'theta': 0.0 },
        { 'frame_name': 'frame5', 'joint_type': 'r', 'link_length': 0.0,  'twist': 0.0,  'offset': 0.02,'theta': 0.0 },
    ],
    robot_name="6DOF")

for i in range(0, 91, 1):
    system('clear')
    robot.f_kin([i, 90, -90, i, 45, i])
    print(robot.get_transforms(6, real=True),'\n')
    print(robot.jacobian())
    print(robot.get_joint_states(),'\n')
    robot.singular_configs_check()
    sleep(0.02)
