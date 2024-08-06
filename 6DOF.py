'''DH TABLE FOR SAMPLE 6DOF ROBOT'''
'''---------+-------------+--------------+---------------+--------------+
|    Link   | Link Length |  Link Twist  |  Joint Offset |     Theta    |
------------+-------------+--------------+---------------+--------------+
|     1     |      a1     |      0 deg   |       d1      |    theta1    |
|     2     |      a2     |      0 deg   |       d2      |    theta2    |
|     3     |      a3     |      0 deg   |       d3      |    theta3    |
|     4     |      a4     |      0 deg   |       d4      |    theta4    |
|     5     |      a5     |      0 deg   |       d5      |    theta5    |
|     6     |      a6     |      0 deg   |       d6      |    theta6    |
+-----------+-------------+--------------+---------------+------------'''

from Libs.RoboKinematics import CreateRobot
import timeit
from time import sleep


#SCARA JOINT CONFIGURATIONS [Frame_name, Joint_type, link_length, link_twist, joint_offset, joint_variable] the joint_variables are initialized to 0.0 here]
dh_params_link1 = ["frame0", "r", 0.05,  90.0,  0.105, 0.0]
dh_params_link2 = ["frame1", "r", 0.14,  0.0,  0.0,  0.0]
dh_params_link3 = ["frame2", "r", 0.17,  90.0,  0.0, 0.0]
dh_params_link4 = ["frame3", "r", 0.0,  -90.0,  0.0, 0.0]
dh_params_link5 = ["frame4", "r", 0.0,  90.0,  0.0, 0.0]
dh_params_link6 = ["frame5", "r", 0.0,  0.0,  0.0, 0.0]


dh_params = [
    dh_params_link1,
    dh_params_link2,
    dh_params_link3,
    dh_params_link4,
    dh_params_link5,
    dh_params_link6,
    ]

_6dof = CreateRobot(dh_params,"6DOF")

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

exec_time("_6dof.move_joints([0,90,-90,0,0,0])")
r =_6dof.move_joints([0,90,-90,0,0,0])
_6dof.print_transforms(6)


