import unittest
from Src.RoboKinematics import CreateKinematicModel
from math import pi
import numpy as np
from numpy.testing import assert_array_equal

class TestRoboKinematics(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        print("set_up_class")
    
    @classmethod
    def tearDownClass(cls):
        print("tear_down_class")
    
    def setUp(self):
        self.dh_params = [
                {"frame_name": "link1", "joint_type": "r", "link_length": 1.0, "twist": 0, "offset": 0, "theta": 0},
                {"frame_name": "link2", "joint_type": "r", "link_length": 1.0, "twist": 0, "offset": 0, "theta": 0}
            ]
        self.scara = CreateKinematicModel(
            self.dh_params,
            robot_name="SCARA", link_twist_in_rads=True,
        )

    def tearDown(self):
        pass

    def test_creat_kinematic_model(self):
        with self.assertRaises(ValueError):
            self.scara = CreateKinematicModel(
                [],
                robot_name="SCARA", link_twist_in_rads=True,
            )
        with self.assertRaises(KeyError):
            dh_params = [
                {"frame_na__": "link1", "joint_type": "r", "link_length": 1.0, "twist": 0, "offset": 0, "theta": 0},
                {"frame_name": "link2", "joint_type": "r", "link_length": 1.0, "twist": 0, "offset": 0, "theta": 0}
            ]
            # Create the kinematic model
            robot = CreateKinematicModel(dh_params, robot_name="2DOF Robot")

        with self.assertRaises(TypeError):
            # Create the kinematic model
            robot = CreateKinematicModel(self.dh_params)

    def test_f_kin(self):

        with self.assertRaises(IndexError):
            self.scara.f_kin([])


        with self.assertRaises(IndexError):
            q = [0]
            self.scara.f_kin(q)


        with self.assertRaises(IndexError):
            q = [0, 0, 0]
            self.scara.f_kin(q)

        joint_angles = [0,0]

        self.scara.f_kin(joint_angles)

        t = self.scara.get_transforms(2)

        expected_result =  [[1., 0., 0., 2.],
                            [0., 1., 0., 0.],
                            [0., 0., 1., 0.],
                            [0., 0., 0., 1.]]
        
        assert_array_equal(t, np.array(expected_result))


    def test_get_transforms(self):

        with self.assertRaises(IndexError):
            self.scara.get_transforms(3)


        with self.assertRaises(IndexError):
            qr = self.scara.set_joints([0, 0, 0])
            t = self.scara.f_kin(qr)
            self.scara.get_transforms(0)


        with self.assertRaises(IndexError):
            qr = self.scara.set_joints([0, 0, 0])
            t = self.scara.f_kin(qr)
            self.scara.get_transforms(6)


        with self.assertRaises(IndexError):
            qr = self.scara.set_joints([0, 0, 0])
            t = self.scara.f_kin(qr)
            self.scara.get_transforms(-1)


    def test_lin_ang_velocity(self):

        with self.assertRaises(TypeError):
            self.scara.f_kin([0, 0])
            self.scara.lin_ang_velocity(['a', 1])


        with self.assertRaises(ValueError):
            self.scara.f_kin([0, 0])
            self.scara.lin_ang_velocity([1])


        with self.assertRaises(ValueError):
            self.scara.f_kin([0, 0])
            self.scara.lin_ang_velocity([1, 1, 1])


        with self.assertRaises(ValueError):
            self.scara.f_kin([0, 0])
            self.scara.lin_ang_velocity([])


    def  test_joint_vels(self):

        with self.assertRaises(TypeError):
            self.scara.f_kin([0, 0])
            self.scara.joint_vels([-0.28, 0, -1, 0, '0', 2, ])

        with self.assertRaises(ValueError):
            self.scara.f_kin([0, 0])
            self.scara.joint_vels([-0.28, 0, -1, 0, 0, 2, 1])

        with self.assertRaises(ValueError):
            self.scara.f_kin([0, 0])
            self.scara.joint_vels([])

    def test_i_kin(self):

        with self.assertRaises(TypeError):

            self.scara.f_kin([0, 0])

            start = self.scara.get_joint_states(rads=True)

            target_position = ['0.96592583', 1.67303261, 0, 0, 0, 1.30899694]

            self.scara.i_kin(target_position)

        with self.assertRaises(IndexError):

            self.scara.f_kin([0, 0])

            start = self.scara.get_joint_states(rads=True)

            target_position = [0.1315, 0.0479, -0.1, 3.1416, 0, 0.3491, 1, 2]

            goal = self.scara.i_kin(target_position)

        with self.assertRaises(IndexError):

            self.scara.f_kin([0, 0])

            self.scara.get_joint_states(rads=True)

            target_position = []

            self.scara.i_kin(target_position)

        joint_angles = [45, 20]

        self.scara.f_kin(joint_angles)

        tr = self.scara.get_transforms(2)

        t = self.scara.SE3(tr, merge_res=True)

        final = self.scara.i_kin(t)

        expected_final = np.deg2rad([45.0, 20.0])

        assert_array_equal(final, expected_final)


if __name__ == '__main__':
    unittest.main()
