import unittest
from Libs.RoboKinematics import CreateKinematicModel
from math import pi

class TestRoboKinematics(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        print("set_up_class")
    
    @classmethod
    def tearDownClass(cls):
        print("tear_down_class")
    
    def setUp(self):
        self.scara = CreateKinematicModel(
            [
                {
                    'frame_name': 'frame0',
                    'joint_type': 'r',
                    'link_length': 0.0,
                    'twist': 0.0,
                    'offset': 0.4,
                    'theta': 0.0
                },
                {
                    'frame_name': 'frame1',
                    'joint_type': 'r',
                    'link_length': 0.14,
                    'twist': pi,
                    'offset': 0.0,
                    'theta': 0.0
                },
                {
                    'frame_name': 'frame2',
                    'joint_type': 'p',
                    'link_length': 0.0,
                    'twist': 0.0,
                    'offset': 0.0,
                    'theta': 0.0
                }
            ],
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
    
    def test_set_joints(self):

        with self.assertRaises(TypeError):
            self.scara.set_joints(["1", 2, 3])
            self.scara.set_joints([1, 2, '3'])

        with self.assertRaises(IndexError):
            self.scara.set_joints([1, 2, 3, 4])
            self.scara.set_joints([1, 2])
            self.scara.set_joints([])

    def test_f_kin(self):
        with self.assertRaises(TypeError):
            self.scara.f_kin([])

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
            qr = self.scara.set_joints([0, 0, 0])
            t = self.scara.f_kin(qr)
            lav = self.scara.lin_ang_velocity(['a', 1, 1])
        with self.assertRaises(ValueError):
            qr = self.scara.set_joints([0, 0, 0])
            t = self.scara.f_kin(qr)
            lav = self.scara.lin_ang_velocity([1, 1])
        with self.assertRaises(ValueError):
            qr = self.scara.set_joints([0, 0, 0])
            t = self.scara.f_kin(qr)
            lav = self.scara.lin_ang_velocity([1, 1, 1, 1])
        with self.assertRaises(ValueError):
            qr = self.scara.set_joints([0, 0, 0])
            t = self.scara.f_kin(qr)
            lav = self.scara.lin_ang_velocity([])


    def  test_joint_vels(self):
        with self.assertRaises(TypeError):
            qr = self.scara.set_joints([0, 0, 0])
            t = self.scara.f_kin(qr)
            self.scara.joint_vels([-0.28, 0, -1, 0, '0', 2, ])
        with self.assertRaises(ValueError):
            qr = self.scara.set_joints([0, 0, 0])
            t = self.scara.f_kin(qr)
            self.scara.joint_vels([-0.28, 0, -1, 0, 0, 2, 1])
        with self.assertRaises(ValueError):
            qr = self.scara.set_joints([0, 0, 0])
            t = self.scara.f_kin(qr)
            self.scara.joint_vels([])

    def test_i_kin(self):
        with self.assertRaises(TypeError):
            qr = self.scara.set_joints([60, 30, 0])
            start = self.scara.get_joint_states(rads=True)
            t = self.scara.f_kin(qr)
            goal = self.scara.i_kin(['0.1315', 0.0479, -0.1, 3.1416, 0, 0.3491])
        with self.assertRaises(IndexError):
            qr = self.scara.set_joints([60, 30, 0])
            start = self.scara.get_joint_states(rads=True)
            t = self.scara.f_kin(qr)
            goal = self.scara.i_kin([0.1315, 0.0479, -0.1, 3.1416, 0, 0.3491, 1, 2])
        with self.assertRaises(IndexError):
            qr = self.scara.set_joints([60, 30, 0])
            start = self.scara.get_joint_states(rads=True)
            t = self.scara.f_kin(qr)
            goal = self.scara.i_kin([])

    def test_ptraj(self):
        with self.assertRaises(TypeError):
            self.scara.ptraj(['0', 0, 0],[10, 20, 30], 1, 0)
        with self.assertRaises(TypeError):
            self.scara.ptraj(['0', 0, 0],[10, '20', 30], 1, 0)
        with self.assertRaises(IndexError):
            self.scara.ptraj([0, 0],[10, 20, 30], 1, 0)
        with self.assertRaises(IndexError):
            self.scara.ptraj([0, 0, 0],[10, 20], 1, 0)
        with self.assertRaises(IndexError):
            self.scara.ptraj([0, 0, 0, 0],[10, 20, 30], 1, 0)
        with self.assertRaises(IndexError):
            self.scara.ptraj([0, 0, 0],[10, 20, 30, 40], 1, 0)
        with self.assertRaises(IndexError):
            self.scara.ptraj([],[10, 20, 30], 1, 0)
        with self.assertRaises(IndexError):
            self.scara.ptraj([0, 0, 0],[], 1, 0)
        with self.assertRaises(IndexError):
            self.scara.ptraj([],[], 1, 0)


if __name__ == '__main__':
    unittest.main()
