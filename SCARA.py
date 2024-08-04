'''DH TABLE FOR SAMPLE SCARA ROBOT ARM'''
'''---------+-------------+--------------+---------------+--------------+
|    Link   | Link Length |  Link Twist  |  Joint Offset |     Theta    |
------------+-------------+--------------+---------------+--------------+
|     1     |      a1     |      0 deg   |       d1      |    theta1    |
|     2     |      a2     |      0 deg   |       d2      |    theta2    |
|     3     |      a3     |      0 deg   |       d3      |    theta3    |
+-----------+-------------+--------------+---------------+------------'''

from Libs.FKin import Create_Transformation

th1 = 60
th2 = 20
d3 = 0.2

#SCARA DH PARAMS LIST FOR LINKS 1,2 AND 3 >> [link_length, link_twist, joint_offset, joint_variable]
link1 = [0.0,  0.0,  0.4,  th1] #Frame0
link2 = [0.14, 180,  0.0,  th2] #Frame1
link3 = [0.0,  0.0,  d3,   0.0] #Frame2

SCARA = [
    link1,
    link2,
    link3,
    ]

_SCARA_arm = Create_Transformation(SCARA)

h0_3 = _SCARA_arm.getTransforms(3)

print("\n")
for i in h0_3:
    print(i)
print("\n")
