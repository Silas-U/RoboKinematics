"""
Author: Silas Udofia
Date: 2024-08-02
Description: This script performs kinematics analysis for a 4DOF robot.

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


'''DH TABLE FOR SAMPLE 4DOF ROBOT'''
'''---------+-------------+--------------+---------------+--------------+
|    Link   | Link Length |  Link Twist  |  Joint Offset |     Theta    |
------------+-------------+--------------+---------------+--------------+
|     1     |      a1     |      0 deg   |       d1      |    theta1    |
|     2     |      a2     |      0 deg   |       d2      |    theta2    |
|     3     |      a3     |      0 deg   |       d3      |    theta3    |
|     4     |      a4     |      0 deg   |       d4      |    theta4    |
+-----------+-------------+--------------+---------------+------------'''


from Libs.RoboKinematics import CreateKinematicModel
from math import pi as π
from time import sleep
from timeit import default_timer as timer


# Creates a kinematic model of the SCARA robot
rb = CreateKinematicModel(
    [
        ("frame0", "r", 0.0,   90.0,  0.1,   0.0),
        ("frame1", "r", 0.14,  0.0,   0.0,   0.0),
        ("frame2", "r", 0.14,  90.0,  0.0,   0.0),
        ("frame3", "r", 0.0,   0.0,   0.0,   0.0),
    ],
    robot_name="4DOF",
)


"""
If joint vars are provided in radians then the min and max limit should be set in radians: 
Default min and max limits are set in degrees
"""

qr = rb.set_joints([-50, 90, -90, 50]) # Default configuration
t = rb.f_kin(qr)
home = rb.get_joint_states(rads=True)
start = timer()
target= rb.i_kin([0.24248711, 0, 0.1,  3.14159265, 0.52359878, 0]) # x,y,z,roll,pitch,yaw
trj = rb.ptraj(home, target , 0.1, 0)
end = timer()
rb.plot(trj)
print('It took %.5f s. to execute.' % (end - start))


