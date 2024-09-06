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

'''DH TABLE FOR SAMPLE 2DOF ROBOT'''
'''---------+-------------+--------------+---------------+--------------+
|    Link   | Link Length |  Link Twist  |  Joint Offset |     Theta    |
------------+-------------+--------------+---------------+--------------+
|     1     |      a1     |      0 deg   |       d1      |    theta1    |
|     2     |      a2     |      0 deg   |       d2      |    theta2    |
+-----------+-------------+--------------+---------------+------------'''


# Creates a kinematic model of the SCARA robot
scara = CreateKinematicModel(
    [
        {
         'frame_name': 'frame0', 
         'joint_type': 'r',
         'link_length': 1.0, 
         'twist': 0.0, 
         'offset': 0.0,
         'theta': 0.0
        },
        {'frame_name': 'frame1', 
         'joint_type': 'r',
         'link_length': 1.0,
         'twist': 0.0,   
         'offset': 0.0,
         'theta': 0.0
         },
    ],
    robot_name="SCARA", link_twist_in_rads=True
)


start = timer()

trj_time = [5]

t =  scara.f_kin([0, 0])
home = scara.get_joint_states(rads=True)
target_1 = scara.i_kin([1,  -1,   0,   0,   0, -90], euler_in_deg=True)

jq = [
    home,
    target_1
]

trajectory = scara.traj_gen(jq, trj_time, 0, plot=True)
end = timer()
print('It took %.5f s. to execute.' % (end - start))

