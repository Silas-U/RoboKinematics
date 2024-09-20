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
from timeit import default_timer as timer

"""
The Denavit–Hartenberg parameters for Puma560  robot is shown below.

Kinematics  theta [rad]     a [m]   d [m]   alpha [rad]
Joint 1     0               0       0.6718      π/2
Joint 2     0               0.4318    0           0
Joint 3     0               0.0203  0.15       -π/2
Joint 4     0               0       0.4318      π/2
Joint 5     0               0       0          -π/2
Joint 6     0               0       0           0
"""

# Creates a kinematic model of the Puma560 robot
Puma560 = CreateKinematicModel(
     [        
        { 'frame_name': 'frame0', 'joint_type': 'r', 'link_length': 0.0, 'twist': 90.0, 'offset': 0.0,'theta': 0.0 },
        { 'frame_name': 'frame1', 'joint_type': 'r', 'link_length': 0.4318, 'twist': 0.0, 'offset': 0.0,'theta': 0.0 },
        { 'frame_name': 'frame2', 'joint_type': 'r', 'link_length': 0.0203, 'twist': -90.0, 'offset': 0.15,'theta': 0.0 },
        { 'frame_name': 'frame3', 'joint_type': 'r', 'link_length': 0.0, 'twist': 90.0, 'offset': 0.4318,'theta': 0.0 },
        { 'frame_name': 'frame4', 'joint_type': 'r', 'link_length': 0.0, 'twist': -90.0, 'offset': 0.0,'theta': 0.0 },
        { 'frame_name': 'frame5', 'joint_type': 'r', 'link_length': 0.0, 'twist': 0.0, 'offset': 0.0,'theta': 0.0 }
     ],
     robot_name="Puma560")


start = timer()

trj_time = [5]

Puma560.f_kin([0, 45, -90, 30, 0, 0])

home = Puma560.get_joint_states(rads=True)

target_1 = Puma560.i_kin([0.26453836, -0.06334258,  0.45908105, -0.35924867, -0.7797434,  1.07086644])

jq = [
    home,
    target_1
]

trajectory = Puma560.traj_gen(jq, trj_time, 0, plot=True)
end = timer()
# print('It took %.5f s. to execute.' % (end - start)) # set plot to False before uncommenting
