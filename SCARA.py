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
from time import sleep


#SCARA JOINT CONFIGURATIONS [Frame_name, Joint_type, link_length, link_twist, joint_offset, joint_variable] the joint_variables are initialized to 0.0 here]
dh_params_link1 = ["frame0", "r", 0.0,  0.0,  0.4, 0.0]
dh_params_link2 = ["frame1", "r", 0.14, 180,  0.0, 0.0]
dh_params_link3 = ["frame2", "p", 0.0,  0.0,  0.0, 0.0]


dh_params = [
    dh_params_link1,
    dh_params_link2,
    dh_params_link3,
    ]

scara_arm = CreateRobot(dh_params,"SCARA")

def exec_time(f):
        print(
            '{:.10f}'.format(
                timeit.timeit(
                f,
                globals=globals(),
                number=1,
            )
        )        
    )


scara_arm.move_joints([0,0,0.2],rads=True)
scara_arm.print_transforms(3)
sleep(2)
scara_arm.move_joints([45,45,0.2])
scara_arm.print_transforms(3)
sleep(2)
scara_arm.move_joints([0,0,0.2])
scara_arm.print_transforms(3)
sleep(2)
scara_arm.move_joints([45,45,0.2])
scara_arm.print_transforms(3)
sleep(2)
scara_arm.move_joints([0,0,0.2])
scara_arm.print_transforms(3)
sleep(2)

# set_joint_limit(min,max)

#Moves the robot revolute_joint_1 to 45 deg, revolute_joint_2 to 20 deg and prismatic_joint_3 to 0.2m
# scara_arm.move_joints([45,20,0.2])
# scara_arm.print_transforms(3)

#print the tcp of the robot
# tcp = scara_arm.get_tcp()
# print(tcp)

#prints the joint origin (xyz) of the robot at joint 3
# jo = scara_arm.get_j_origin(3)
# print(jo)

#prints the rotation matrix R_03
# rm = scara_arm.get_r_matrix(3) 
# print(rm)

#gets the homogeneous transformation h0_3 matrix in list format
# h0_3 = scara_arm.get_transforms(3)

#Prints the execution time of a function
#exec_time("scara_arm.get_transforms(3)")




